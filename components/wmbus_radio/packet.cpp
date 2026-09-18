#include "packet.h"

#include <algorithm>
#include <ctime>

#include "esphome/components/wmbus_common/meters.h"
#include "esphome/core/helpers.h"

#include "decode3of6.h"

#define WMBUS_PREAMBLE_SIZE (3)
#define WMBUS_MODE_C_SUFIX_LEN (2)
#define WMBUS_MODE_C_PREAMBLE (0x54)
#define WMBUS_BLOCK_A_PREAMBLE (0xCD)
#define WMBUS_BLOCK_B_PREAMBLE (0x3D)

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "packet";
Packet::Packet() { this->data_.reserve(WMBUS_PREAMBLE_SIZE); }

// Determine the link mode based on the first byte of the data
LinkMode Packet::link_mode() {
  if (this->link_mode_ == LinkMode::UNKNOWN) {
    if (this->data_.size()) {
      if (this->data_[0] == WMBUS_MODE_C_PREAMBLE) {
        this->link_mode_ = LinkMode::C1;
      } else {
        this->link_mode_ = LinkMode::T1;
      }
    }
  }

  return this->link_mode_;
}

void Packet::set_rssi(int8_t rssi) { this->rssi_ = rssi; }

// Get value of L-field
uint8_t Packet::l_field() {
  switch (this->link_mode()) {
  case LinkMode::C1:
    if (this->data_.size() > 2)
      return this->data_[2];
    break;
  case LinkMode::T1: {
    auto decoded = decode3of6(this->data_);
    if (decoded && !decoded->empty())
      return (*decoded)[0];
    break;
  }
  default:
    break;
  }
  return 0;
}

size_t Packet::expected_size() {
  if (!this->expected_size_) {
    // Format A
    //   L-field = length without CRC fields and without L (1 byte)
    // Format B
    //   L-field = length with CRC fields and without L (1 byte)
    auto l_field = this->l_field();

    // The 2 first blocks contains 25 bytes when excluding CRC and the L-field
    // The other blocks contains 16 bytes when excluding the CRC-fields
    // Less than 26 (15 + 10)
    auto nrBlocks = l_field < 26 ? 2 : (l_field - 26) / 16 + 3;

    // Add all extra fields, excluding the CRC fields + 2 CRC bytes for each
    // block
    auto nrBytes = l_field + 1 + 2 * nrBlocks;

    if (this->link_mode() != LinkMode::C1) {
      this->expected_size_ = encoded_size(nrBytes);
    } else if (this->data_.size() > 1 && this->data_[1] == WMBUS_BLOCK_A_PREAMBLE) {
      this->expected_size_ = WMBUS_MODE_C_SUFIX_LEN + nrBytes;
    } else if (this->data_.size() > 1 && this->data_[1] == WMBUS_BLOCK_B_PREAMBLE) {
      this->expected_size_ = WMBUS_MODE_C_SUFIX_LEN + 1 + l_field;
    }
  }
  ESP_LOGV(TAG, "expected_size: %zu", this->expected_size_);
  return this->expected_size_;
}

size_t Packet::rx_capacity() {
  // TODO: Remove side effects?
  auto cap = this->data_.capacity() - this->data_.size();
  this->data_.resize(this->data_.capacity());
  return cap;
}

uint8_t *Packet::rx_data_ptr() {
  return this->data_.data() + this->data_.size();
}

bool Packet::calculate_payload_size() {
  auto total_length = this->expected_size();
  this->data_.reserve(total_length);
  return total_length;
}

std::optional<Frame> Packet::convert_to_frame() {
  std::optional<Frame> frame = {};

  // Raw 3-of-6 bytes of a T1 packet, kept for the format B retry below.
  std::vector<uint8_t> t1_encoded;

  ESP_LOGD(TAG, "Have data from radio (%zu bytes)", this->data_.size());
  debugPayload("raw packet", this->data_);

  if (this->expected_size() == this->data_.size()) {
    if (this->link_mode() == LinkMode::T1) {
      // T1 carries both frame formats and the format is not visible up front.
      // expected_size() sizes the read for format A, which is the longer of the
      // two, so a format B frame is followed by trailing noise — and noise does
      // not decode, making decode3of6() reject the whole buffer. Keep the
      // encoded bytes so format B can be retried on a prefix.
      t1_encoded = this->data_;
      this->frame_format_ = "A";
      auto decoded_data = decode3of6(this->data_);
      if (decoded_data)
        this->data_ = decoded_data.value();
    } else if (this->link_mode() == LinkMode::C1) {
      if (this->data_.size() > 1) {
        if (this->data_[1] == WMBUS_BLOCK_A_PREAMBLE)
          this->frame_format_ = "A";
        else if (this->data_[1] == WMBUS_BLOCK_B_PREAMBLE)
          this->frame_format_ = "B";
      }
      if (this->data_.size() >= WMBUS_MODE_C_SUFIX_LEN) {
        this->data_.erase(this->data_.begin(),
                          this->data_.begin() + WMBUS_MODE_C_SUFIX_LEN);
      }
    } else {
      ESP_LOGE(TAG, "unknown link mode!");
    }
  } else {
    ESP_LOGE(TAG, "expected_size: %zu NOT size: %zu", this->expected_size(),
             this->data_.size());
  }

  bool crcOk = false;

  if (this->frame_format_ == "A") {
    crcOk = trimCRCsFrameFormatA(this->data_);

    if (!crcOk && !t1_encoded.empty()) {
      // Retry as format B. Its L-field counts the CRCs, so the frame is L + 1
      // bytes; decode only that prefix, leaving the trailing noise out.
      uint8_t l_field = 0;

      if (t1_encoded.size() >= 3) {
        std::vector<uint8_t> head(t1_encoded.begin(), t1_encoded.begin() + 3);
        auto head_decoded = decode3of6(head);
        if (head_decoded && !head_decoded->empty())
          l_field = head_decoded->front();
      }

      size_t b_encoded_size = encoded_size(static_cast<size_t>(l_field) + 1);

      if (l_field >= 11 && b_encoded_size <= t1_encoded.size()) {
        std::vector<uint8_t> b_part(t1_encoded.begin(),
                                    t1_encoded.begin() + b_encoded_size);
        auto b_decoded = decode3of6(b_part);

        if (b_decoded && trimCRCsFrameFormatB(b_decoded.value())) {
          this->data_ = std::move(b_decoded.value());
          this->frame_format_ = "B";
          crcOk = true;
          ESP_LOGV(TAG, "T1 frame decoded as format B (%zu bytes)",
                   this->data_.size());
        }
      }
    }
  } else if (this->frame_format_ == "B") {
    crcOk = trimCRCsFrameFormatB(this->data_);
  }

  // Diagnostika: když T1 rámec neprojde ani jako A, ani jako B, vypiš syrová
  // 3-of-6 data, ať se dá formát určit offline místo odhadem. Jen prvních pár
  // rámců po startu, aby to nezahltilo log.
  if (!crcOk && !t1_encoded.empty()) {
    static int dumped = 0;
    if (dumped < 3) {
      dumped++;
      ESP_LOGW(TAG, "T1 REJECT #%d: %zu encoded bytes, dump follows", dumped,
               t1_encoded.size());
      for (size_t off = 0; off < t1_encoded.size(); off += 48) {
        size_t n = std::min<size_t>(48, t1_encoded.size() - off);
        std::vector<uint8_t> chunk(t1_encoded.begin() + off,
                                   t1_encoded.begin() + off + n);
        ESP_LOGW(TAG, "T1 REJECT #%d [%3zu]: %s", dumped, off,
                 format_hex(chunk).c_str());
      }
    }
  }

  int dummy;
  if (crcOk && (checkWMBusFrame(this->data_, (size_t *)&dummy, &dummy, &dummy, false) ==
                FrameStatus::FullFrame))
    frame.emplace(this);

  delete this;

  return frame;
}

Frame::Frame(Packet *packet)
    : data_(std::move(packet->data_)), link_mode_(packet->link_mode_),
      rssi_(packet->rssi_), format_(packet->frame_format_) {}

std::vector<uint8_t> &Frame::data() { return this->data_; }
LinkMode Frame::link_mode() { return this->link_mode_; }
int8_t Frame::rssi() { return this->rssi_; }
std::string Frame::format() { return this->format_; }

std::vector<uint8_t> Frame::as_raw() { return this->data_; }
std::string Frame::as_hex() { return format_hex(this->data_); }
std::string Frame::as_rtlwmbus() {
  const size_t time_repr_size = sizeof("YYYY-MM-DD HH:MM:SS.00Z");
  char time_buffer[time_repr_size] = "1970-01-01 00:00:00.00Z";
  auto t = std::time(NULL);
  auto *tm_info = std::gmtime(&t);
  if (tm_info != nullptr) {
    std::strftime(time_buffer, time_repr_size, "%F %T.00Z", tm_info);
  }

  auto output = std::string{};
  output.reserve(2 + 5 + 24 + 1 + 4 + 5 + 2 * this->data_.size() + 1);

  output += linkModeName(this->link_mode_); // size 2
  output += ";1;1;";                        // size 5
  output += time_buffer;                    // size 24
  output += ';';                            // size 1
  output += std::to_string(this->rssi_);    // size up to 4
  output += ";;;0x";                        // size 5
  output += this->as_hex();                 // size 2 * frame.size()
  output += "\n";                           // size 1

  return output;
}

void Frame::mark_as_handled() { this->handlers_count_++; }
uint8_t Frame::handlers_count() { return this->handlers_count_; }

} // namespace wmbus_radio
} // namespace esphome
/*
Self-contained reader for Apollo Cyber RT record files (cyber_recorder output).

No Apollo runtime and no libprotobuf: the record container and the
apollo.drivers.PointCloud payload are decoded with a minimal protobuf
wire-format parser. Field numbers follow the vendored definitions in
proto/record.proto and proto/pointcloud.proto (copied from the deployed
apollo-lite checkout).

File layout (cyber/record/file/record_file_reader.cc):
  [Section{int32 type; pad; int64 size}] [proto::Header, `size` bytes]
  ... header block is padded to sizeof(Section) + 2048 bytes ...
  repeated sections:
    SECTION_CHANNEL(4)      -> proto::Channel {name=1, message_type=2}
    SECTION_CHUNK_HEADER(1) -> proto::ChunkHeader (skipped)
    SECTION_CHUNK_BODY(2)   -> proto::ChunkBody {repeated SingleMessage=1}
    SECTION_INDEX(3)        -> end of data
  SingleMessage {channel_name=1, time=2, content=3}
  apollo.drivers.PointCloud {frame_id=2, point=4 (repeated PointXYZIT),
                             measurement_time=5}
  PointXYZIT {x=1 float, y=2 float, z=3 float, intensity=4 uint32,
              timestamp=5 uint64}

Only COMPRESS_NONE records are supported (cyber_recorder's default); a
compressed record fails loudly instead of mis-parsing.
*/

#ifndef FAST_CALIB_CYBER_RECORD_READER_HPP
#define FAST_CALIB_CYBER_RECORD_READER_HPP

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "log.h"

namespace cyber_record {

struct ApolloPoint {
  float x = 0.f, y = 0.f, z = 0.f;
  float intensity = 0.f;
};

struct ApolloPointCloud {
  std::string frame_id;
  double measurement_time = 0.0;
  std::vector<ApolloPoint> points;
};

// ---------------------------------------------------------------------------
// Minimal protobuf wire-format decoding
// ---------------------------------------------------------------------------
namespace wire {

struct Slice {
  const uint8_t* p;
  const uint8_t* end;
  bool empty() const { return p >= end; }
};

inline bool readVarint(Slice& s, uint64_t& out) {
  out = 0;
  int shift = 0;
  while (s.p < s.end && shift < 64) {
    uint8_t b = *s.p++;
    out |= (uint64_t)(b & 0x7F) << shift;
    if (!(b & 0x80)) return true;
    shift += 7;
  }
  return false;
}

inline bool readTag(Slice& s, uint32_t& field, uint32_t& type) {
  uint64_t key;
  if (!readVarint(s, key)) return false;
  field = (uint32_t)(key >> 3);
  type = (uint32_t)(key & 0x7);
  return true;
}

inline bool readLenDelim(Slice& s, Slice& out) {
  uint64_t len;
  if (!readVarint(s, len)) return false;
  if ((uint64_t)(s.end - s.p) < len) return false;
  out.p = s.p;
  out.end = s.p + len;
  s.p += len;
  return true;
}

inline bool readFixed32(Slice& s, uint32_t& out) {
  if ((size_t)(s.end - s.p) < 4) return false;
  std::memcpy(&out, s.p, 4);  // record files are little-endian, as are all
  s.p += 4;                   // supported targets (x86_64 / aarch64)
  return true;
}

inline bool readFixed64(Slice& s, uint64_t& out) {
  if ((size_t)(s.end - s.p) < 8) return false;
  std::memcpy(&out, s.p, 8);
  s.p += 8;
  return true;
}

inline bool skipField(Slice& s, uint32_t type) {
  switch (type) {
    case 0: {  // varint
      uint64_t v;
      return readVarint(s, v);
    }
    case 1: {  // fixed64
      uint64_t v;
      return readFixed64(s, v);
    }
    case 2: {  // length-delimited
      Slice sub;
      return readLenDelim(s, sub);
    }
    case 5: {  // fixed32
      uint32_t v;
      return readFixed32(s, v);
    }
    default:
      return false;  // groups (3/4) never appear in these protos
  }
}

}  // namespace wire

// ---------------------------------------------------------------------------
// Payload decoding
// ---------------------------------------------------------------------------
namespace detail {

inline bool parsePointXYZIT(wire::Slice s, ApolloPoint& pt, bool& valid) {
  float x = std::nanf(""), y = std::nanf(""), z = std::nanf("");
  uint64_t intensity = 0;
  uint32_t field, type;
  while (!s.empty()) {
    if (!wire::readTag(s, field, type)) return false;
    uint32_t f32;
    uint64_t v;
    switch (field) {
      case 1:  // x
        if (!wire::readFixed32(s, f32)) return false;
        std::memcpy(&x, &f32, 4);
        break;
      case 2:  // y
        if (!wire::readFixed32(s, f32)) return false;
        std::memcpy(&y, &f32, 4);
        break;
      case 3:  // z
        if (!wire::readFixed32(s, f32)) return false;
        std::memcpy(&z, &f32, 4);
        break;
      case 4:  // intensity
        if (!wire::readVarint(s, v)) return false;
        intensity = v;
        break;
      default:
        if (!wire::skipField(s, type)) return false;
    }
  }
  // drop NaN and zero returns (same policy as the python extraction path)
  valid = (x == x && y == y && z == z) &&
          !(std::abs(x) < 1e-6f && std::abs(y) < 1e-6f && std::abs(z) < 1e-6f);
  if (valid) {
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = (float)intensity;
  }
  return true;
}

inline bool parsePointCloud(wire::Slice s, ApolloPointCloud& out) {
  uint32_t field, type;
  while (!s.empty()) {
    if (!wire::readTag(s, field, type)) return false;
    switch (field) {
      case 2: {  // frame_id
        wire::Slice sub;
        if (!wire::readLenDelim(s, sub)) return false;
        out.frame_id.assign((const char*)sub.p, sub.end - sub.p);
        break;
      }
      case 4: {  // repeated PointXYZIT
        wire::Slice sub;
        if (!wire::readLenDelim(s, sub)) return false;
        ApolloPoint pt;
        bool valid = false;
        if (!parsePointXYZIT(sub, pt, valid)) return false;
        if (valid) out.points.push_back(pt);
        break;
      }
      case 5: {  // measurement_time (double)
        uint64_t v;
        if (!wire::readFixed64(s, v)) return false;
        std::memcpy(&out.measurement_time, &v, 8);
        break;
      }
      default:
        if (!wire::skipField(s, type)) return false;
    }
  }
  return true;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// Record container walking
// ---------------------------------------------------------------------------
class Reader {
 public:
  // Streams every PointCloud message on `channel` from the record files (in
  // the given order) into `cb`, stopping after `max_frames` frames (0 = all).
  // Returns the number of frames delivered; channels_seen collects every
  // channel name found, for error reporting.
  static size_t ReadPointClouds(
      const std::vector<std::string>& files, const std::string& channel,
      size_t max_frames, const std::function<void(ApolloPointCloud&&)>& cb,
      std::set<std::string>* channels_seen = nullptr) {
    size_t frames = 0;
    for (const auto& f : files) {
      if (max_frames && frames >= max_frames) break;
      if (!readOneFile(f, channel, max_frames, frames, cb, channels_seen))
        LOG_WARN("[Record] Failed to fully parse %s (stopping at %zu frames).",
                 f.c_str(), frames);
    }
    return frames;
  }

 private:
  static constexpr int kSectionLength = 16;   // sizeof(struct Section)
  static constexpr int kHeaderLength = 2048;  // cyber HEADER_LENGTH

  static bool readOneFile(const std::string& path, const std::string& channel,
                          size_t max_frames, size_t& frames,
                          const std::function<void(ApolloPointCloud&&)>& cb,
                          std::set<std::string>* channels_seen) {
    std::ifstream fin(path, std::ios::binary);
    if (!fin.is_open()) {
      LOG_ERROR("[Record] Cannot open %s", path.c_str());
      return false;
    }

    int32_t type;
    int64_t size;
    if (!readSection(fin, type, size) || type != 0 /*SECTION_HEADER*/) {
      LOG_ERROR("[Record] %s is not a cyber record file.", path.c_str());
      return false;
    }
    std::vector<uint8_t> buf((size_t)size);
    if (!fin.read((char*)buf.data(), size)) return false;
    uint64_t compress = 0;
    parseHeaderCompress({buf.data(), buf.data() + buf.size()}, compress);
    if (compress != 0) {
      LOG_ERROR(
          "[Record] %s uses compressed chunks (compress=%llu); only "
          "COMPRESS_NONE records are supported.",
          path.c_str(), (unsigned long long)compress);
      return false;
    }
    fin.seekg(kSectionLength + kHeaderLength, std::ios::beg);

    while (readSection(fin, type, size)) {
      if (type == 3 /*SECTION_INDEX*/) break;
      if (max_frames && frames >= max_frames) return true;
      if (type == 2 /*SECTION_CHUNK_BODY*/ ||
          (channels_seen && type == 4 /*SECTION_CHANNEL*/)) {
        buf.resize((size_t)size);
        if (!fin.read((char*)buf.data(), size)) return false;
        wire::Slice s{buf.data(), buf.data() + buf.size()};
        if (type == 4) {
          parseChannelName(s, channels_seen);
        } else if (!parseChunkBody(s, channel, max_frames, frames, cb)) {
          return false;
        }
      } else {
        fin.seekg(size, std::ios::cur);
      }
      if (!fin.good()) break;
    }
    return true;
  }

  static bool readSection(std::ifstream& fin, int32_t& type, int64_t& size) {
    uint8_t hdr[kSectionLength];
    if (!fin.read((char*)hdr, kSectionLength)) return false;
    std::memcpy(&type, hdr, 4);  // 4 bytes padding between type and size
    std::memcpy(&size, hdr + 8, 8);
    return size >= 0;
  }

  static void parseHeaderCompress(wire::Slice s, uint64_t& compress) {
    uint32_t field, type;
    while (!s.empty()) {
      if (!wire::readTag(s, field, type)) return;
      if (field == 3 && type == 0) {  // Header.compress
        wire::readVarint(s, compress);
        return;
      }
      if (!wire::skipField(s, type)) return;
    }
  }

  static void parseChannelName(wire::Slice s, std::set<std::string>* out) {
    uint32_t field, type;
    while (!s.empty()) {
      if (!wire::readTag(s, field, type)) return;
      if (field == 1 && type == 2) {  // Channel.name
        wire::Slice sub;
        if (!wire::readLenDelim(s, sub)) return;
        out->insert(std::string((const char*)sub.p, sub.end - sub.p));
        continue;
      }
      if (!wire::skipField(s, type)) return;
    }
  }

  // ChunkBody { repeated SingleMessage messages = 1; }
  static bool parseChunkBody(wire::Slice s, const std::string& channel,
                             size_t max_frames, size_t& frames,
                             const std::function<void(ApolloPointCloud&&)>& cb) {
    uint32_t field, type;
    while (!s.empty()) {
      if (!wire::readTag(s, field, type)) return false;
      if (field != 1 || type != 2) {
        if (!wire::skipField(s, type)) return false;
        continue;
      }
      wire::Slice msg;
      if (!wire::readLenDelim(s, msg)) return false;

      // SingleMessage {channel_name=1, time=2, content=3}
      std::string name;
      wire::Slice content{nullptr, nullptr};
      uint32_t mfield, mtype;
      while (!msg.empty()) {
        if (!wire::readTag(msg, mfield, mtype)) return false;
        if (mfield == 1 && mtype == 2) {
          wire::Slice sub;
          if (!wire::readLenDelim(msg, sub)) return false;
          name.assign((const char*)sub.p, sub.end - sub.p);
        } else if (mfield == 3 && mtype == 2) {
          if (!wire::readLenDelim(msg, content)) return false;
        } else if (!wire::skipField(msg, mtype)) {
          return false;
        }
      }
      if (name != channel || content.p == nullptr) continue;

      ApolloPointCloud cloud;
      if (!detail::parsePointCloud(content, cloud)) {
        LOG_WARN("[Record] Skipping a message on %s that failed to parse as "
                 "apollo.drivers.PointCloud.", channel.c_str());
        continue;
      }
      cb(std::move(cloud));
      if (++frames, max_frames && frames >= max_frames) return true;
    }
    return true;
  }
};

}  // namespace cyber_record

#endif  // FAST_CALIB_CYBER_RECORD_READER_HPP

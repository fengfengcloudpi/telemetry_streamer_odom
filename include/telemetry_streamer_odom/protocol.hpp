#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include "telemetry_streamer_odom/config.hpp"

// 协议常量
static constexpr uint32_t UDP_MAGIC = 0xABCD1234;
static constexpr uint8_t  UDP_VER   = 1;

static constexpr uint8_t MSG_TYPE_STREAM = 0x0C; // 普通STREAM帧
static constexpr uint8_t TMPL_STATE_ACTIVE = 0;

// 报文头定义 (打包时用1字节对齐)
#pragma pack(push, 1)
struct UdpHdr {
    uint32_t magic;
    uint8_t  ver;
    uint8_t  msg_type;
    uint8_t  flags;
    uint8_t  rsv0;
    uint32_t seq;
    uint64_t ts_usec;
    uint16_t payload_len; // bytes after this header
    uint16_t rsv1;
    uint32_t crc32; // 占位: 简单校验
};

struct TemplateFrameHead {
    uint16_t template_id;
    uint16_t template_ver;
    uint8_t  role;
    uint8_t  status;
    uint8_t  flags;
    uint8_t  tmpl_state;
    uint16_t lease_ms;
    uint16_t remain;
    uint16_t n_int;
    uint16_t n_float;
    uint16_t n_str;
    uint16_t throttle_hz;
    uint32_t rsv;
};
#pragma pack(pop)

// 占位CRC32（请按需求替换为真正CRC）
inline uint32_t fake_crc32(const uint8_t* data, size_t len) {
    uint32_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c = (c * 131u) + data[i];
    }
    return c;
}

// 将 float 槽打成一帧 UDP 数据包 (STREAM)
struct PackedDatagram {
    std::vector<uint8_t> bytes;
};

inline PackedDatagram build_stream_frame(
    uint16_t template_id,
    uint16_t template_ver,
    uint64_t ts_usec,
    uint32_t seq,
    StreamBuffers tStreamBuffers
) {
    TemplateFrameHead tfh{};
    tfh.template_id   = template_id;
    tfh.template_ver  = template_ver; // 版本/哈希，可先写1
    tfh.role          = 0; // DATA
    tfh.status        = 0; // OK
    tfh.flags         = 0x02; // 假设 bit1=HAS_FLOAT
    tfh.tmpl_state    = TMPL_STATE_ACTIVE;
    tfh.lease_ms      = 0;
    tfh.remain        = 0; // 0=无限
    tfh.n_int         = 0;
    tfh.n_float       = static_cast<uint16_t>(tStreamBuffers.floats.size());
    tfh.n_str         = 0;
    tfh.throttle_hz   = 0;
    tfh.rsv           = 0;

    // TemplateFrameHead 后直接接 float[]
    uint16_t payload_len =
        static_cast<uint16_t>( sizeof(TemplateFrameHead)
                             + tStreamBuffers.floats.size()*sizeof(float) );

    UdpHdr hdr{};
    hdr.magic       = UDP_MAGIC;
    hdr.ver         = UDP_VER;
    hdr.msg_type    = MSG_TYPE_STREAM;
    hdr.flags       = 0;
    hdr.rsv0        = 0;
    hdr.seq         = seq;
    hdr.ts_usec     = ts_usec;
    hdr.payload_len = payload_len;
    hdr.rsv1        = 0;
    hdr.crc32       = 0; // 暂时0，稍后回填

    PackedDatagram out;
    out.bytes.resize(sizeof(UdpHdr) + payload_len);

    // copy hdr
    std::memcpy(out.bytes.data(), &hdr, sizeof(hdr));
    // copy TemplateFrameHead
    std::memcpy(out.bytes.data()+sizeof(hdr), &tfh, sizeof(tfh));
    // copy float payload
    if (!tStreamBuffers.floats.empty()) {
        std::memcpy(
            out.bytes.data()+sizeof(hdr)+sizeof(tfh),
            tStreamBuffers.floats.data(),
            tStreamBuffers.floats.size()*sizeof(float)
        );
    }

      // copy float payload
    if (!tStreamBuffers.ints.empty()) {
        std::memcpy(
            out.bytes.data()+sizeof(hdr)+sizeof(tfh)+tStreamBuffers.floats.size()*sizeof(float),
            tStreamBuffers.ints.data(),
            tStreamBuffers.ints.size()*sizeof(int)
        );
    }

    return out;
}




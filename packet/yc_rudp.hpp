#pragma once
#include <algorithm>
#include <vector>
#include <functional>
#include <variant>
#include <chrono>

#include <mutex>

#include "packets.hpp"
#include "yc_packet.hpp"

namespace yc_rudp {


    
    
    // 받아야할 패킷 번호 보다 뒷 번호 인지 확인 하는 함수.
    bool is_later_seq(int seq1, int seq2) {
        seq1 += seq1 < ACK_COUNTER_MAX / 2 ? ACK_COUNTER_MAX : 0;
        seq2 += seq2 < ACK_COUNTER_MAX / 2 ? ACK_COUNTER_MAX : 0;
        return seq1 > seq2;
    }
    
    struct packet_raw {
        char* buf;
        std::vector<char> data;
        size_t len;
        volatile bool is_used;
    };
    
    class rudp_packet_receiver {
        std::vector<packet_raw> packets;
        std::vector<bool> loss_packets;
        std::function<void(int)> send_ack;
        std::function<void(int)> recv_ack_callback;
        packet_raw nodelay_packet;
        int last_idx = 0;
        std::mutex recv_mutex;
    public:
        rudp_packet_receiver(
            std::function<void(int)> send_ack_,
            std::function<void(int)> recv_ack_callback_)
            :
        send_ack(std::move(send_ack_)),
        recv_ack_callback(std::move(recv_ack_callback_))
        {
            packets.reserve(ACK_COUNTER_MAX);
            for (int i = 0; i < ACK_COUNTER_MAX; i++) {
                packets.push_back({ nullptr, std::vector<char>(512), 0, false });
            }
            loss_packets.reserve(ACK_COUNTER_MAX);
            for (int i = 0; i < ACK_COUNTER_MAX; i++) {
                loss_packets.push_back(false);
            }
        }
        
    public:
        inline std::vector<packet_raw*> packet_read(const char* data, size_t size) {
            const packet_ack_type ack = data[0];
            yc_pack::udp::convert_ack data_to_ack;
            data_to_ack.load(ack);

            if(!data_to_ack.use_ack) {
                nodelay_packet.buf = const_cast<char*>(data) + 1;
                nodelay_packet.len = size;
                return { &nodelay_packet };
            }
            if(data_to_ack.is_ack_packet) {
                if(data_to_ack.counter >= ACK_COUNTER_MAX) return {};
                recv_ack_callback(data_to_ack.counter);
                return {};
            }
            auto lidx = last_idx;

            const auto c = data_to_ack.counter;
            if(c < lidx || packets[c].is_used) return {};
            
            send_ack(c);

            // server가 이미 받았던 패킷을 다시 보냈다는 것을 검사하는 코드
            if(!loss_packets[c] && is_later_seq(lidx, c)) {
                return {};
            }
            
            loss_packets[c] = false;

#if _MSVC_LANG < 202004L
            bool is_loss = std::ranges::find(loss_packets, true) != loss_packets.end();
#else
            bool is_loss = std::ranges::contains(loss_packets, true);
#endif 

            if(lidx == c && !is_loss) {
                std::vector<packet_raw*> r = {};
                r.reserve(ACK_COUNTER_MAX);
                int i = c;

                packets[c].buf = const_cast<char*>(data) + 1;
                packets[c].len = size;
                packets[c].is_used = false;
                
                r.emplace_back(&packets[c]);
                ++i %= ACK_COUNTER_MAX;
                while(packets[i].is_used) {
                    r.emplace_back(&packets[i]);
                    packets[i].is_used = false;
                    ++i %= ACK_COUNTER_MAX;
                }
                {
                    std::lock_guard lock(recv_mutex);
                    last_idx = i;
                }
                return r;
            }
            std::copy_n(data, size, packets[c].data.data());
            packets[c].buf = packets[c].data.data() + 1;
            packets[c].len = size;
            packets[c].is_used = true;

            for(int j = lidx; j < c; ++j) loss_packets[j] = true;
            return {};
        }
    };
    struct send_packet_raw : packet_raw {
        int64_t timestamp;
        bool is_resend_packet;
    };
    class rudp_packet_sender {
        std::vector<send_packet_raw> packets;
        
        std::function<bool(char*, int)> sendto_func;
        std::function<void(int)> send_complate_func;
        std::function<void(int64_t)> timeover_event;

        int64_t rtt = 100;
        int idx = 0;
        std::mutex idx_mutex;

        volatile bool is_timeout = false;

        [[nodiscard]] int64_t get_timestamp() const {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }

    public:
        rudp_packet_sender( std::function<bool(char*, int)> sendto_func_,
                            std::function<void(int)> send_complate_func_,
                            std::function<void(int64_t)> timeover_event_)
        : sendto_func(std::move(sendto_func_)),
          send_complate_func(std::move(send_complate_func_)),
          timeover_event(std::move(timeover_event_))
        {
            packets.reserve(ACK_COUNTER_MAX);
            for (int i = 0; i < ACK_COUNTER_MAX; i++) {
                packets.push_back({ nullptr, std::vector<char>(512), 0, false });
            }
        }

        void reset() {
            rtt = 100;
            idx = 0;
            is_timeout = false;
            for(auto& i : packets) i.is_used = false;
        }
        inline bool get_return_ack(int count) {
            auto& p = packets[count];
            if(!p.is_used) return false;
            send_complate_func(count);
            p.is_used = false;

            rtt = p.is_resend_packet ? rtt
                                     : static_cast<int>((get_timestamp() - p.timestamp) * 1.5f);
            
            return true;
        }
        int send_ack_packet(int count) {
            if(is_timeout) return -1;
            yc_pack::udp::convert_ack ack(count);
            ack.is_ack_packet = true;
            char buf = ack.to_ack();
            return sendto_func(&buf, 1) ? count : -1;
        }
        int send_packet(const char* data, size_t size, bool use_ack = false) {
            if(is_timeout) return -1;
            int idx_ = -1;
            {
                std::lock_guard lock(idx_mutex);
                idx_ = idx;
                idx = (idx + 1) % ACK_COUNTER_MAX;
            }
            auto& packet = packets[idx_];
            if(packet.is_used) return -1;

            packet.buf = packet.data.data();
            
            yc_pack::udp::convert_ack ack(yc_pack::udp::convert_ack::ack_type::no_ack);
            if(use_ack) {
                ack.use_ack = use_ack;
                ack.counter = idx;
                ack.is_ack_packet = false;
            }
            packet.data[0] = ack.to_ack();
            
            std::copy_n(const_cast<char*>(data), size, packet.data.data() + 1);
            packet.len = size + 1;
            packet.is_used = true;
            packet.timestamp = get_timestamp();
            packet.is_resend_packet = false;
            return sendto_func(packet.buf, static_cast<int>(size) + 1) ? (idx_ ? idx_ - 1 : ACK_COUNTER_MAX - 1) : -1;
        }

        void resend_run() {
            if(is_timeout) return;
            
            for(auto& i : packets) {
                if(!i.is_used) continue;
                if(const auto t = get_timestamp();
                    i.timestamp + rtt < t) {
                    i.is_resend_packet = true;
                    sendto_func(i.buf, static_cast<int>(i.len));
                    i.timestamp = t;
                    rtt = static_cast<int>(rtt * 1.5f);
                    if(rtt > 1000) {
                        i.is_used = false;
                        is_timeout = true;
                        timeover_event(rtt);
                    }
                }
            }
        }
    };
}
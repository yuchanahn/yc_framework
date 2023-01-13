#pragma once
#include <algorithm>
#include <vector>
#include <functional>
#include <variant>
#include <chrono>

#include <mutex>

#include "yc_packet.hpp"

namespace yc_rudp
{
    struct packet_raw {
        char data[PACKET_SIZE_MAX] {};
        packet_size_type len {};
        volatile bool is_used {};
    };
    struct receive_packet_raw {
        char data[PACKET_SIZE_MAX] {};
        packet_size_type len {};
    };

    struct send_packet_raw : packet_raw {
        int64_t timestamp{};
        bool is_resend_packet{};
    };

    struct rudp_buffer_t {
        std::vector<packet_raw> pkt_buffer;
        std::vector<receive_packet_raw> receive_buffer;
        std::vector<send_packet_raw> send_buffer;
        
        rudp_buffer_t(const int io_thread_count)
                        : pkt_buffer(ACK_COUNTER_MAX * io_thread_count * 2)
                        , receive_buffer(ACK_COUNTER_MAX * io_thread_count * 2)
                        , send_buffer(ACK_COUNTER_MAX + 1) {
        }
    };
    
    [[nodiscard]] inline int64_t get_timestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    
    
    // 받아야할 패킷 번호 보다 뒷 번호 인지 확인 하는 함수.
    inline bool is_later_seq(int seq1, int seq2) {
        seq1 += seq1 < ACK_COUNTER_MAX / 2 ? ACK_COUNTER_MAX : 0;
        seq2 += seq2 < ACK_COUNTER_MAX / 2 ? ACK_COUNTER_MAX : 0;
        return seq1 > seq2;
    }

    //패킷에서 seq를 뽑아내는 함수
    inline int get_seq(const char* buf) {
        yc_pack::udp::convert_ack ack;
        ack.load(*buf);
        return ack.use_ack ? ack.counter : -1;
    }
    
    //ack처리용 패킷인지를 확인 하는 함수
    inline bool is_ack_packet(const char* buf) {
        yc_pack::udp::convert_ack ack;
        ack.load(*buf);
        return ack.is_ack_packet;
    }

    inline int get_next_seq(const int seq) {
        return (seq + 1) % ACK_COUNTER_MAX;
    }

    inline int make_seq(const int seq) {
        return seq % ACK_COUNTER_MAX;
    }
    int each_seq(const int start, const int end, auto f) {
        if (start == end) return end;
        for(int i = start; i < end; ++i) f(i % ACK_COUNTER_MAX);
        return end % ACK_COUNTER_MAX;
    }
    
    /**
     * \brief 패킷 버퍼에 패킷을 넣는 함수
     * \param pkt_buf 패킷이 담길 버퍼 buf -> [t1 | t2... || t1(no ack) | t2(no ack)...]
     * \param thread_id 현재 스레드의 id
     * \param thread_cnt_max 스레드의 최대 개수
     * \param buf 패킷의 데이터
     * \param len 패킷의 길이
     * \return 성공시 pushed index, 실패시 -1
     */
    inline int push_packet(
        std::vector<packet_raw>& pkt_buf,
        const int thread_id,
        const int thread_cnt_max,
        const char* buf,
        const size_t len
        ) {
        // 만약 ack 패킷이라면 ack 완료 통지.
        // 함수 바깥에서 처리해줘야 한다.
        const auto seq = get_seq(buf);
        int idx = -1;
        if(seq == -1) {
            const auto s = ACK_COUNTER_MAX * thread_cnt_max + ACK_COUNTER_MAX * thread_id;
            for(int i = 0; i < ACK_COUNTER_MAX; ++i) {
                if(!pkt_buf[i + s].is_used) {
                    idx = i + s;
                    break;
                }
            }
        } else {
            idx = thread_id * ACK_COUNTER_MAX + seq;
        }
        auto& [data, pkt_len, is_used] = pkt_buf[idx];

        //이미 사용중인 패킷이면 실패
        if(is_used) return -1;

        std::copy_n(buf + 1, len - 1, data);
        pkt_len = static_cast<packet_size_type>(len - 1);
        is_used = true;
        
        return idx;
    }
    
    inline packet_raw* find_seq_of_packets(
        std::vector<packet_raw>& pkt_buf,
        const int thread_id,
        const int seq
        ) {
        return pkt_buf[thread_id * ACK_COUNTER_MAX + seq].is_used ? &pkt_buf[thread_id * ACK_COUNTER_MAX + seq] : nullptr;
    }
    
    /**
     * \brief 패킷을 읽어 올 범위를 가져오는 함수
     * 이 함수는 보호 되어야 한다.
     * 아니면 하나의 스레드에서만 실행된 다는 것이 보장되어야 한다.
     * \param pkt_buf 사용할 패킷 버퍼
     * \param recv_buf [OUT] 읽어온 패킷을 담을 버퍼
     * \param no_ack_read_buf [OUT] ack 패킷이 아닌 패킷을 담을 버퍼
     * \param thread_cnt_max 스레드의 최대 개수
     * \param end 읽어온 패킷의 마지막 위치
     * \return 읽을 패킷의 범위. 실패시 (-1, -1) 반환. 실패시 buffer를 비워서 패킷을 버린다.
     */
    inline std::pair<int, int> get_read_range(
        std::vector<packet_raw>& pkt_buf,
        std::vector<receive_packet_raw>& recv_buf,
        std::vector<receive_packet_raw>& no_ack_read_buf,
        const int thread_cnt_max,
        int end
        ) {
        int start = end;
        int cnt = 0;
        for(int i = start; i < start + ACK_COUNTER_MAX; ++i) {
            packet_raw* ptr = nullptr;
            for(int thread_id = 0; thread_id < thread_cnt_max; ++thread_id) {
                ptr = find_seq_of_packets(pkt_buf, thread_id, i % ACK_COUNTER_MAX);
                if(ptr) break;
            }
            if(ptr == nullptr) {
                end = i;
                break;
            }
            const int idx = i % ACK_COUNTER_MAX;
            std::copy_n(ptr->data, ptr->len, recv_buf[idx].data);
            recv_buf[idx].len = ptr->len;
            ptr->is_used = false;
            cnt++;
        }
        
        for(int thread_id = 0; thread_id < thread_cnt_max; ++thread_id) {
            for(int i = 0; i < ACK_COUNTER_MAX; ++i) {
                auto& [data, len, is_used] = pkt_buf[thread_cnt_max * ACK_COUNTER_MAX + thread_id * ACK_COUNTER_MAX + i];
                if(!is_used) break;
                no_ack_read_buf.emplace_back();
                std::copy_n(data, len, no_ack_read_buf.back().data);
                no_ack_read_buf.back().len = len;
                is_used = false;
            }
        }
        if(cnt > ACK_COUNTER_MAX) return {-1, -1};
        return { start, end };
    }

    /**
     * \brief packet에 ack header를 붙입니다.
     * 함수를 호출 한 뒤 바로 send 해야 합니다.
     * \param send_buf [OUT] ack 패킷을 담을 버퍼
     * \param buf 패킷 몸체
     * \param len 패킷의 길이
     * \param use_ack ack를 사용할지 여부
     * \param seq ack 패킷의 seq
     * \return ack 패킷의 seq, 버퍼가 전부 차있을 경우 -1, ack를 사용하지 않는 패킷일 경우 ACK_COUNTER_MAX
     */
    inline int ready_to_send(
        std::vector<send_packet_raw>& send_buf,
        char* buf,
        const int len,
        const bool use_ack,
        const int seq
        ){
        if(use_ack && send_buf[seq].is_used) return -1;

        yc_pack::udp::convert_ack ack;
        ack.use_ack = use_ack;
        ack.is_ack_packet = false;
        ack.counter = seq;
        const int s = use_ack ? seq : ACK_COUNTER_MAX;
        send_buf[s].data[0] = ack.to_ack();
        std::copy_n(buf, len, send_buf[s].data + 1);
        send_buf[s].timestamp = get_timestamp();
        send_buf[s].len = static_cast<packet_size_type>(len + 1);
        send_buf[s].is_used = use_ack;
        
        return s;
    }

    /**
     * \brief ack을 받았을 때 호출합니다.
     * \param send_buf [OUT] send 완료 처리를 위한 버퍼
     * \param rtt [OUT] rtt새로 계산한 값
     * \param seq ack 패킷의 seq
     * \return send 완료 처리한 버퍼의 index, 실패시 -1
     */
    inline int set_send_complete(
        std::vector<send_packet_raw>& send_buf,
        int& rtt,
        const int seq
        ) {
        if(!send_buf[seq].is_used) return -1;
        
        send_buf[seq].is_used = false;
        return seq;
    }

    /**
     * \brief 재전송이 필요한 패킷을 찾습니다. rtt의 최소값은 10입니다.
     * \param send_buf [OUT] resend가 필요한지 검사하는 버퍼
     * \param rtt [OUT] new rtt, timeout이 발생했을 경우 -1
     * \param timeout timeout을 발생 시킬 최소값 (ms)
     * \param seq 최신의 seq
     * \return resend가 필요한 패킷의 seq, 없을 경우 0
     */
    inline std::vector<int> get_resend_packets(
        std::vector<send_packet_raw>& send_buf,
        int& rtt,
        const int timeout
        ) {
        std::vector<int> ret;
        
        for (int i = 0; i < ACK_COUNTER_MAX; ++i) {
            auto& pkt = send_buf[i];
            if (!pkt.is_used) continue;
            if (const auto t = get_timestamp(); pkt.timestamp + rtt < t) {
                ret.push_back(i);
                pkt.is_resend_packet = true;
                //pkt.timestamp = t;
                rtt = static_cast<int>(rtt * 1.5f);
                rtt = std::max(rtt, 10);
                if (rtt > timeout) {
                    pkt.is_used = false;
                    rtt = -1;
                    return {};
                }
            }
        }
        return ret;
    }
}

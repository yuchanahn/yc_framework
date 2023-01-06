#include "packet/yc_rudp.hpp"
#include "thread_pool.hpp"
#include "test_module/yc_test.hpp"
#include "thread/nto_memory.hpp"

int main(int argc, char* argv[]) {
    constexpr int io_thread_count = 4;
    
    yc_rudp::rudp_buffer_t rudp(io_thread_count);
    std::vector<yc_rudp::receive_packet_raw> no_ack_packets;

    char buf[512] {};
    auto push = [&](int seq, std::string data, int thread_id) {
        yc_pack::udp::convert_ack ack;
        ack.use_ack = true;
        ack.counter = seq;
        ack.is_ack_packet = false;
    
        buf[0] = ack.to_ack();
        std::copy_n(data.c_str(), data.length(), buf + 1);
        push_packet(rudp.pkt_buffer, thread_id % io_thread_count, io_thread_count, buf, data.length()+1);
    };
    auto get_read_range_for_buffer = std::bind_front(
        yc_rudp::get_read_range,
        std::ref(rudp.pkt_buffer),
        std::ref(rudp.receive_buffer)
        );
    int m_seq = -1;
    
    for(int i = 0; i < 22; ++i) {
        push(m_seq, std::format("[1]number : {}", m_seq), rand() % io_thread_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        m_seq = yc_rudp::get_next_seq(m_seq);
    }
    int end = 0;
    {
        auto[s, e] = get_read_range_for_buffer(no_ack_packets, io_thread_count, end);
    
        for(;s < e; ++s) {
            printf("%s\n", rudp.receive_buffer[yc_rudp::make_seq(s)].data);
        }
        end = yc_rudp::make_seq(e);
    }
    for(int i = 0; i < 40; ++i) {
        push(m_seq, std::format("[2]number : {}", m_seq), rand() % io_thread_count);
        m_seq = yc_rudp::get_next_seq(m_seq);
    }
    {
        auto[s, e] = get_read_range_for_buffer(no_ack_packets, io_thread_count, end);
        
        for(;s < e; ++s) {
            printf("%s\n", rudp.receive_buffer[yc_rudp::make_seq(s)].data);
        }
        end = yc_rudp::make_seq(e);
        
    }

    for(int i = 0; i < 36; ++i) {
        push(m_seq, std::format("[3]number : {}", m_seq), rand() % io_thread_count);
        m_seq = yc_rudp::get_next_seq(m_seq);
    }
    
    {
        auto[s, e] = get_read_range_for_buffer(no_ack_packets, io_thread_count, end);
        for(;s < e; ++s) {
            printf("%s\n", rudp.receive_buffer[yc_rudp::make_seq(s)].data);
        }
        end = yc_rudp::make_seq(e);
    }
    int rtt = 100;
    int seq = -1;

    test_thread_pool pool(10);
    auto send_test = [&](int s, int ms, bool use_ack = true) {
        const std::string str = "hello" + std::to_string(s);
        std::copy_n(str.c_str(), str.length(), buf);
        ready_to_send(rudp.send_buffer, buf, str.length(), use_ack, s);

        if(use_ack) {
            pool.add_task([&rtt, &rudp, &seq, s, ms] {
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
                seq = set_send_complete(rudp.send_buffer, rtt, s);
                printf(" # send complete %d\n", seq);
            });
        }
    };
    
    send_test(0, 10);
    send_test(1, 30);
    send_test(2, 50);
    send_test(3, 350);
    send_test(4, 151);
    send_test(5, 2350, false);
    send_test(5, 1001);
    
    while(true) {
        auto r = get_resend_packets(rudp.send_buffer, rtt, 1000);
        for(auto& i : r) {
            printf("resend_data : %s\n", rudp.send_buffer[i].data + 1);
        }
        if(r.size()) printf("rtt [%d], size vector : %lld\n", rtt, r.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pool.wait_all();
    return 0;
}
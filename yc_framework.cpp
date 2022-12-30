#include "packet/yc_rudp.hpp"
#include "packet/yc_packet.hpp"

#include <functional>
#include <ranges>
#include <atomic>

namespace yc {

    struct task_data_t {
        std::thread thread;
        std::atomic_bool stop;
        std::function<void()> func;
        int id;
    };
    std::vector<task_data_t*> yc_tasks;

    class task {
    public:
        int id = -1;
        std::function<void()> body;
        task(std::function<void()> body_) : body(body_) {
            if(yc_tasks.empty()) {
                auto data = new task_data_t;
                data->stop = false;
                data->thread = std::thread([data] {
                    while(!data->stop) {
                        data->func();
                    }
                });
                yc_tasks.push_back(data);
                data->id = yc_tasks.size() - 1;
                id = data->id;
            } else {
                for(const auto& i : yc_tasks | std::views::filter([](task_data_t* x) { return x->stop.load(); })
                                             | std::views::take(1)) {
                    i->func = body;
                    id = i->id;
                    i->stop = false;
                }
            }
        }
        ~task() {
            yc_tasks[id]->stop = true;
        }
    };
    
}

int main(int argc, char* argv[]) {
    auto sendto = [](char* buf, int len) {
        if(len == 1) printf("send ack!\n");
        else         printf("sendto : %s[%d]\n", buf + 1, len);
        return true;
    };
    auto send_suceess = [](int seq) {
        printf("send suceess : %d\n", seq);
    };
    
    yc_rudp::rudp_packet_sender sender(sendto, send_suceess, [](int64_t rtt){ printf("is timeout [rtt : %lld]", rtt);});
    yc_rudp::rudp_packet_receiver receiver(
    [&sender](int x) {
        sender.send_ack_packet(x);
    }, [&sender](int x) {
        printf("get ack packet!!! %d\n", sender.get_return_ack(x));
    });
    /*
    std::string spacket = "packet_test";
    sender.send_packet(spacket.c_str(), spacket.length() + 1, true);

    auto get_ack_f = [&](int c) {
        yc_pack::udp::convert_ack tack_0(c);
        tack_0.is_ack_packet = true;
        auto p = tack_0.to_ack();
        receiver.packet_read(reinterpret_cast<const char*>(&p), 1);
    };

    std::thread t([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(222));
        get_ack_f(0);
    });

    */
    //while(true) {
    //    sender.resend_run();
    //    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //}

    auto read_packet_f = [&](int c) {
        yc_pack::udp::convert_ack tack_0(c);
        tack_0.is_ack_packet = false;
        tack_0.counter = c;
        char buf[5];
        buf[0] = tack_0.to_ack();

        // print bit
        for(int i = 0; i < 8; ++i) {
            printf("%d", (buf[0] >> (7 - i)) & 1);
        }
        printf("\n");
        
        std::copy_n(std::format("C:{}", c).c_str(), 4, &buf[1]);
        for(auto& i : receiver.packet_read(buf, 5)) printf("read packet : %s\n", i->buf);
    };
    
    read_packet_f(0);
    read_packet_f(2);
    read_packet_f(1);
    read_packet_f(5);
    read_packet_f(4);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    read_packet_f(3);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100000));
    return 0;
}
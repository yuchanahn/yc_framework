#include "packet/yc_rudp.hpp"
#include "thread_pool.hpp"
#include "test_module/yc_test.hpp"
#include "thread/nto_memory.hpp"

int main(int argc, char* argv[]) {

    char buf[] = "packet_test";
    // 패킷 검사
    yc_pack::pkt_vrfct(buf, 4);
    
    return 0;
}
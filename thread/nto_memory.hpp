#pragma once
#include <vector>
#include <ranges>

//yc class to hold the data
#define YC_USE

template <typename Nto>
concept nto_type = requires {
    YC_USE Nto::thread_id;
};

/**
 * \brief threads... <-> one thread only (main_thread?)
 * 다수의 스레드와 하나의 스레드가 lock free 하게 객체를 관리 할 수 있는 클래스입니다.
 * 예상하는 대로 동작하였을 경우. thread safe 함을 확인 ^^.
 * [*** 주의 ***] object를 read하는 thread는 한개여야 합니다.
 * [*** 주의 ***] object를 write하는 thread는 여러개 일 수 있습니다.
 * \tparam Type Type of thread safe object
 * \tparam ThreadCount Number of threads, default is 1
 * \tparam Max Maximum number of threads, default is 1
 */
template <nto_type Type, int ThreadCount = 1, int Max = 8196>
class nto_memory {
    std::vector<int> index_q_[ThreadCount];
    std::vector<int> r_index_q_;
    std::vector<Type> mem_;
    std::vector<Type*> mem_ptr_;
    volatile bool used_[Max * ThreadCount] {};
    
    int find_first_false(int thread_id) {
        auto& iq = index_q_[thread_id];
        if(iq.empty()) {
            for(int i = 0; i < Max; ++i) {
                if(!used_[thread_id * Max + i]) iq.push_back(Max * thread_id + i);
            }
        }
        if(iq.empty()) return -1;
        auto i = iq.back();
        iq.pop_back();
        return i;
    }
public:
    explicit nto_memory() : mem_(Max * ThreadCount), mem_ptr_(Max * ThreadCount) {
        for (int i = 0; i < ThreadCount; ++i) {
            for (int j = 0; j < Max; ++j) 
                mem_[i * Max + j].thread_id = i;
            index_q_[i].reserve(Max);
        }
    }
    bool try_push(const int thread_id, Type*& out) {
        const int idx = find_first_false(thread_id);
        if(idx == -1) return false;
        out = &mem_[idx];
        return true;
    }

    /**
     * \brief 이 함수를 실행했다면 더이상 obj를 참조해서는 안됍니다.
     * \param obj read thread가 권한을 가질 수 있게 되는 객체
     */
    void make_readable(Type*& obj) {
        auto idx = (obj - mem_.data()) % Max;
        used_[obj->thread_id * Max + idx] = true;
        obj = nullptr;
    }

    /**
     * \brief read thread 에서 객체에 참조를 그만합니다.
     * 함수를 호출하고 나서 반환한 obj에 절대로 접근해서는 안됍니다.
     * \param obj write thread 가 객체에 쓸 수 있도록 객체를 반환합니다.
     */
    void read_end(Type*& obj) {
        auto idx = (obj - mem_.data()) % Max;
        used_[obj->thread_id * Max + idx] = false;
        obj = nullptr;
    }
    
    auto& get_read_ranges() {
        mem_ptr_.clear();
        for(auto& i : std::views::filter(used_, [](auto& b) { return b; })) {
            mem_ptr_.push_back(&mem_[&i - used_]);
        }
        return mem_ptr_;
    }
};


/* example
 ===============================

 
struct user_info {
    int id;
    int endpoint;
    YC_USE int thread_id;
};

int main(int argc, char* argv[]) {
    nto_memory<user_info, 4> mem;
    user_info* ptr;
    
    mem.try_push(0, ptr);
    ptr->id = 1;
    ptr->endpoint = 2;
    mem.make_readable(ptr);

    for(auto i : mem.get_read_ranges()) {
        printf("%d %d\n", i->id, i->endpoint);
        mem.read_end(i);
    }
}
    ===============================
    */

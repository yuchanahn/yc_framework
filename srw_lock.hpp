#pragma once
#include<Windows.h>

class srw_lock {
    SRWLOCK srwlock;
public:
    srw_lock() {
        InitializeSRWLock(&srwlock);
    }
    ~srw_lock() {
    }
    void w_lock() {
        AcquireSRWLockExclusive(&srwlock);
    }
    void w_unlock() {
        ReleaseSRWLockExclusive(&srwlock);
    }
    void r_lock() {
        AcquireSRWLockShared(&srwlock);
    }
    void r_unlock() {
        ReleaseSRWLockShared(&srwlock);
    }
};

class r_srw_lock_guard {
    srw_lock& lock;
public:
    r_srw_lock_guard(srw_lock& lock) :lock(lock) {
        lock.r_lock();
    }
    ~r_srw_lock_guard() {
        lock.r_unlock();
    }
};

class w_srw_lock_guard {
    srw_lock& lock;
public:
    w_srw_lock_guard(srw_lock& lock) :lock(lock) {
        lock.w_lock();
    }
    ~w_srw_lock_guard() {
        lock.w_unlock();
    }
};
/**
 * @file LibMultiSense/details/utility/Thread.hh
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) Utility.h file, which was developed under project
 * RD1013.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_THREAD_HH
#define CRL_MULTISENSE_THREAD_HH

#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <string.h>
#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>

#include <vector>
#include <deque>

#include "Exception.hh"

namespace crl {
namespace multisense {
namespace details {
namespace utility {

//
// Forward declarations.

class ScopedLock;

//
// A simple class to wrap pthread creation and joining

class Thread {
public:

    static const uint32_t FLAGS_DETACH = (1 << 0);

    Thread(void    *(*functionP)(void *),
           void    *contextP=NULL,
           uint32_t flags=0,    
           int32_t  scheduler=-1,
           int32_t  priority=0) : m_flags(flags) {

        pthread_attr_t tattr;        
        pthread_attr_init(&tattr);

        //
        // -1 means the user wants default scheduling behavior

        if (-1 != scheduler) {
            struct sched_param sattr = {0};

            //
            // Set our scheduling policy

            if (0 != pthread_attr_setschedpolicy(&tattr, scheduler))
                CRL_EXCEPTION("pthread_attr_setschedpolicy(scheduler=%d) failed: %s",
                                   scheduler, strerror(errno));
            //
            // Set our scheduling parameters (just priority)

            sattr.sched_priority = priority;
            if (0 != pthread_attr_setschedparam(&tattr, &sattr))
                CRL_EXCEPTION("pthread_attr_setschedparam(pri=%d) failed: %s", 
                                   priority, strerror(errno));
            //
            // We must set EXPLICIT_SCHED so the parent's scheduler is not 
            // automatically inherited

            if (0 != pthread_attr_setinheritsched(&tattr, PTHREAD_EXPLICIT_SCHED))
                CRL_EXCEPTION("pthread_attr_setinheritsched(explicit) failed: %s", 
                                   strerror(errno));
        }

        //
        // Create detached, if asked to do so

        if (FLAGS_DETACH & m_flags && 
            0 != pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED))
            CRL_EXCEPTION("pthread_attr_setdetachstate() failed: %s", strerror(errno));
        
        //
        // Finally, create the thread

        if (0 != pthread_create(&m_id, &tattr, functionP, contextP))
            CRL_EXCEPTION("pthread_create() failed: %s", strerror(errno));
    };

    ~Thread() {
        if (!(m_flags & FLAGS_DETACH) &&
            0 != pthread_join(m_id, NULL))
            CRL_DEBUG("pthread_join() failed: %s\n", strerror(errno));
    };          
    
private:

    uint32_t  m_flags;
    pthread_t m_id;
};

//
// A simple mutex class

class Mutex {
public:
    friend class ScopedLock;
    
    Mutex() : m_mutex() {
        if (0 != pthread_mutex_init(&m_mutex, NULL))
            CRL_EXCEPTION("pthread_mutex_init() failed: %s",
                          strerror(errno));
    }

    ~Mutex() {
        pthread_mutex_destroy(&m_mutex);
    };

private:
    pthread_mutex_t m_mutex;
};
    
//
// A simple scoped lock class

class ScopedLock
{
public:

    ScopedLock(Mutex& mutex) {
        this->lock(&mutex.m_mutex);
    };

    ScopedLock(pthread_mutex_t *lockP) {
        this->lock(lockP);
    };

    ScopedLock(pthread_mutex_t& lock) {
        this->lock(&lock);
    };
        
    ~ScopedLock() {
        pthread_mutex_unlock(m_lockP);
    };

private:

    void lock(pthread_mutex_t *lockP) {
        m_lockP = lockP;
        pthread_mutex_lock(m_lockP);
    };
        
    pthread_mutex_t *m_lockP;
};

// A futex-based semaphore.
//
// This implementation does not work across processes.

class Semaphore {
public:

    //
    // Wait for a post (decrement). If thread contention,
    // we may wake up, but be unable to snatch
    // the bait.. hence the while loop.

    bool wait() {        
        do {
            if (0 == wait_())
                return true;
        } while (1);
    };

    //
    // Wait for a post, retrying until timeout

    bool timedWait(const double& timeout) {

        if (timeout < 0.0)
            CRL_EXCEPTION("invalid timeout: %f", timeout);

        struct timespec ts;
        ts.tv_sec  = timeout;
        ts.tv_nsec = (timeout - ts.tv_sec) * 1e9;
        
        do {
            int32_t ret = wait_(&ts);

            if (0 == ret)
                return true;
            else if (ETIMEDOUT == ret)
                return false;

        } while (1);
    };

    //
    // Post to the semaphore (increment.) Here we
    // signal the futex to wake up any waiters.
    
    bool post() {

        //
        // Limit the posts, if asked to do so

        if (m_maximum > 0 && m_avail >= m_maximum)
            return false;
        
        const int32_t nval = __sync_add_and_fetch(&m_avail, 1);
        if (m_waiters > 0)
            syscall(__NR_futex, &m_avail, FUTEX_WAKE, nval, NULL, 0, 0);

        return true;
    };

    //
    // Decrement the semaphore to zero in one-shot.. may
    // fail with thread contention, returns true if
    // successful
    
    bool clear() {
        int32_t val = m_avail;
        if (val > 0)
            return __sync_bool_compare_and_swap(&m_avail, val, 0);
        return true;
    };

    int32_t count    () { return m_avail;   };
    int32_t waiters  () { return m_waiters; };
    bool    decrement() { return wait();    };
    bool    increment() { return post();    };

    Semaphore(std::size_t max=0) :
        m_maximum(max),
        m_avail(0),
        m_waiters(0) {};
    
    ~Semaphore() {};
    
private:

    //
    // This actually does the synchronized decrement if possible, and goes
    // to sleep on the futex if not.

    inline int32_t wait_(const struct timespec *tsP=NULL) {
        
        //
        // Can we decrement the requested amount? If so, return success
        
        const int32_t val = m_avail;
        if (val >= 1 && __sync_bool_compare_and_swap(&m_avail, val, val - 1))
            return 0;

        //
        // We must go to sleep until someone increments. Also keep track of
        // how many threads are waiting on this futex.

        __sync_fetch_and_add(&m_waiters, 1);
        const int32_t ret = syscall(__NR_futex, &m_avail, FUTEX_WAIT, val, tsP, 0, 0);
        __sync_fetch_and_sub(&m_waiters, 1);

        //
        // If we just woke up on the futex, then return EAGAIN, so we
        // can come back in here and attempt a synchronized decrement again.

        if (ETIMEDOUT == ret || -1 == ret) // hmmm.. timeouts are returning -1
            return ETIMEDOUT;
        else
            return EAGAIN;
    };
    
    typedef int32_t aligned_int32_t __attribute__((aligned (4))); // unnecessary ?
    
    const std::size_t m_maximum;
    aligned_int32_t   m_avail;
    aligned_int32_t   m_waiters;
};

//
// A templatized variable signaler

template<class T> class WaitVar {
public:

    void post(const T& data) {
        {
            ScopedLock lock(m_lock);
            m_val = data;
        }
        m_sem.post();
    };

    bool wait(T& data) {
        m_sem.wait();
        {
            ScopedLock lock(m_lock);
            data = m_val;
        }
        return true;
    };

    bool timedWait(T& data,
                   const double& timeout) {

        if (false == m_sem.timedWait(timeout))
            return false;
        {
            ScopedLock lock(m_lock);
            data = m_val;
        }
        return true;
    }
    
    //
    // Use a semaphore with max value of 1. The WaitVar will 
    // either be in a signaled state, or not.

    WaitVar() : m_val(),
                m_lock(),
                m_sem(1) {};

private:

    T                 m_val;
    Mutex     m_lock;
    Semaphore m_sem;
};

//
// A templatized wait queue

template <class T> class WaitQueue {
public:

    void post(const T& data) {
        bool postSem=true;
        {
            ScopedLock lock(m_lock);

            //
            // Limit deque size, if requested

            if (m_maximum > 0 && 
                m_maximum == m_queue.size()) {

                //
                // If at max entries, we will pop_front the oldest,
                // push_back the newest, and leave the semaphore alone

                m_queue.pop_front();
                postSem = false;
            }

            m_queue.push_back(data);
        }
        if (postSem) 
            m_sem.post();
    };

    void kick() {
        m_sem.post();
    };

    bool wait(T& data) {
        m_sem.wait();
        {
            ScopedLock lock(m_lock);

            if (0 == m_queue.size())
                return false;
            else {
                data = m_queue.front();
                m_queue.pop_front();
                return true;
            }
        }
    }

    uint32_t waiters() { 
        return m_sem.waiters();
    };

    uint32_t size() {
        ScopedLock lock(m_lock);
        return m_queue.size();
    }
        
    void clear() {
        ScopedLock lock(m_lock);
        m_queue.clear();
        while(false == m_sem.clear());
    }       

    WaitQueue(std::size_t max=0) : 
        m_maximum(max) {};

private:

    const std::size_t m_maximum;
    std::deque<T>     m_queue;
    Mutex             m_lock;
    Semaphore         m_sem;
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_THREAD_HH */

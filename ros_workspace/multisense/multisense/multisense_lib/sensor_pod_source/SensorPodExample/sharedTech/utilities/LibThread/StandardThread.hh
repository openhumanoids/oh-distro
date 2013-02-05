/**
 * @file LibThread/StandardThread.hh
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
 * Significant history (date, user, job code, action):
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_LIBTHREAD_STANDARDTHREAD_HH
#define CRL_LIBTHREAD_STANDARDTHREAD_HH


#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <string.h>
#include <linux/futex.h>
#include <sys/syscall.h>

#include <vector>
#include <deque>

#include <LibPlatform/StandardException.hh>


namespace crl {

    // Forward declarations.
    class ScopedLock;

    
    // A simple class to wrap pthread creation and joining

    class StandardThread {
    public:

        static const uint32_t FLAGS_DETACH = (1 << 0);

        StandardThread(void    *(*functionP)(void *),
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
                    STANDARD_EXCEPTION("pthread_attr_setschedpolicy(scheduler=%d) failed: %s",
                                       scheduler, strerror(errno));
                //
                // Set our scheduling parameters (just priority)

                sattr.sched_priority = priority;
                if (0 != pthread_attr_setschedparam(&tattr, &sattr))
                    STANDARD_EXCEPTION("pthread_attr_setschedparam(pri=%d) failed: %s", 
                                       priority, strerror(errno));
                //
                // We must set EXPLICIT_SCHED so the parent's scheduler is not 
                // automatically inherited

                if (0 != pthread_attr_setinheritsched(&tattr, PTHREAD_EXPLICIT_SCHED))
                    STANDARD_EXCEPTION("pthread_attr_setinheritsched(explicit) failed: %s", 
                                       strerror(errno));
            }

            //
            // Create detached, if asked to do so

            if (FLAGS_DETACH & m_flags && 
                0 != pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_DETACHED))
                STANDARD_EXCEPTION("pthread_attr_setdetachstate() failed: %s", strerror(errno));
        
            //
            // Finally, create the thread

            if (0 != pthread_create(&m_id, &tattr, functionP, contextP))
                STANDARD_EXCEPTION("pthread_create() failed: %s", strerror(errno));
        };

        ~StandardThread() {
            if (!(m_flags & FLAGS_DETACH) &&
                0 != pthread_join(m_id, NULL))
                STANDARD_DEBUG("pthread_join() failed: %s\n", strerror(errno));
        };          
    
    private:

        uint32_t  m_flags;
        pthread_t m_id;
    };


    // A simple mutex class

    class StandardMutex {
    public:
        friend class ScopedLock;
        
        /**
         * Constructor.
         */
        StandardMutex() : m_mutex() {
            pthread_mutex_init(&m_mutex, NULL);
        }
            
        /**
         * Destructor.
         */
        virtual
        ~StandardMutex() {}

    private:
        pthread_mutex_t m_mutex;
    };
    
    
    // A simple scoped lock class

    class ScopedLock
    {
    public:

        ScopedLock(StandardMutex& mutex) {
            this->lock(&mutex.m_mutex);
        };

        ScopedLock(pthread_mutex_t *lockP) {
            this->lock(lockP);
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



    // A simple futex-based semaphore.
    //
    // This implementation does not work across processes.

    class StandardSemaphore {
    public:

        inline void wait(int32_t count=1) {

            int32_t val;

            if (count <= 0)
                STANDARD_EXCEPTION("invalid wait() count: %d", count);

            do {
                val = m_avail;
                if (val >= count && __sync_bool_compare_and_swap(&m_avail, val, val - count))
                    return;
                __sync_fetch_and_add(&m_waiters, 1);
                syscall(__NR_futex, &m_avail, FUTEX_WAIT, val, NULL, 0, 0);
                __sync_fetch_and_sub(&m_waiters, 1);
            } while (1);
        };

        inline void post(int32_t count=1) {

            if (count <= 0)
                STANDARD_EXCEPTION("invalid post() count: %d", count);

            int32_t nval = __sync_add_and_fetch(&m_avail, count);
            if (m_waiters > 0)
                syscall(__NR_futex, &m_avail, FUTEX_WAKE, nval, NULL, 0, 0);
        };

        inline int32_t getCount()   { return m_avail;   };
        inline int32_t getWaiters() { return m_waiters; };
        inline void    clear()      { m_avail = 0;      };  // is this safe?

        StandardSemaphore() :
            m_avail(0),
            m_waiters(0) {};

        ~StandardSemaphore() {};

    private:

        typedef int32_t aligned_int32_t __attribute__((aligned (4)));
    
        aligned_int32_t m_avail;
        aligned_int32_t m_waiters;
    };


    // A simple wait queue

    template <class T> class WaitQueue {
    public:

        void post(const T& data) {
            {
                ScopedLock lock(&m_lock);
                m_queue.push_back(data);
            }
            m_sem.post();
        };

        bool wait(T& data) {
            m_sem.wait();
            {
                ScopedLock lock(&m_lock);

                if (0 == m_queue.size())
                    return false;
                else {
                    data = m_queue.front();
                    m_queue.pop_front();
                    return true;
                }
            }
        }

        uint32_t getWaiters() {
            return m_sem.getWaiters();
        };

        uint32_t getQueueSize() {
            ScopedLock lock(&m_lock);
            return m_queue.size();
        }
        
        void clear() {
            ScopedLock lock(&m_lock);
            m_queue.clear();
            m_sem.clear();
        }       

        WaitQueue() {
            pthread_mutex_init(&m_lock, NULL);
        };

        ~WaitQueue() {
            pthread_mutex_destroy(&m_lock);
        };

    private:

        std::deque<T>     m_queue;
        pthread_mutex_t   m_lock;
        StandardSemaphore m_sem;
    };

} // namespace crl

#endif /* #ifndef CRL_LIBTHREAD_STANDARDTHREAD_HH */

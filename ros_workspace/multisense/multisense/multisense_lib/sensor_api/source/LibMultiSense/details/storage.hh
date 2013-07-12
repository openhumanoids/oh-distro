/**
 * @file LibMultiSense/details/storage.hh
 *
 * Copyright 2013
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
 *   2012-05-08, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_details_storage_hh
#define LibMultiSense_details_storage_hh

#include "details/utility/Thread.hh"

#include <map>
#include <set>

namespace crl {
namespace multisense {
namespace details {
    
    //
    // A message storage interface
    //
    // Assumes a 1:1 relationship between template class and ID

    class MessageMap {
    public:

        template<class T> void store(const T& msg) {
            utility::ScopedLock lock(m_lock);

            Map::iterator it = m_map.find(MSG_ID(T::ID));
            if (m_map.end() != it) {
                it->second.destroy<T>();
                m_map.erase(it);
            }

            m_map[MSG_ID(T::ID)] = Holder::Create<T>(msg);
        };

        template<class T> Status extract(T& msg) {
            utility::ScopedLock lock(m_lock);

            Map::iterator it = m_map.find(MSG_ID(T::ID));
            if (m_map.end() == it)
                return Status_Error;

            it->second.extract(msg);
            m_map.erase(it);

            return Status_Ok;
        };
        
    private:

        class Holder {
        public:
       
            Holder(void *r=NULL) : m_refP(r) {};
            
            template<class T> static Holder Create(const T& msg) {
                return Holder(reinterpret_cast<void *>(new T(msg)));
            };
      
            template<class T> void destroy() {
                if (NULL == m_refP)
                    CRL_EXCEPTION("destroying NULL reference");
                delete reinterpret_cast<T*>(m_refP);
            };
        
            template<class T> void extract(T& msg) {
                if (NULL == m_refP)
                    CRL_EXCEPTION("extracting NULL reference");
                msg = *(reinterpret_cast<T*>(m_refP));
                destroy<T>();
            };
            
        private:
            void *m_refP;
        };

        typedef std::map<wire::IdType, Holder> Map;

        utility::Mutex m_lock;
        Map            m_map;
    };

    //
    // A constant-depth cache.
    //
    // Up to [depth] entries will be cached.  Oldest entries
    // will be dropped on insertion once full.
    //
    // The age of an entry is determined by std::set<KEY>.lower_bound([min]).
    //
    // Entries may be prevented from deletion by using take(KEY), and
    // re-considered for deletion by using give(KEY)
    //
    // For *_nolock() variations, lock-management must be 
    // done by the user.

    template<class KEY, class DATA>
    class DepthCache {
    public:
        
        DepthCache(std::size_t depth, KEY min) : 
            m_depth(depth),
            m_minimum(min) {};

        ~DepthCache() {
            utility::ScopedLock lock(m_lock);
            m_set.clear();

            typename MapType::iterator it = m_map.begin();
            for(; it != m_map.end();) {
                delete it->second;
                m_map.erase(it++);
            }
        };

        utility::Mutex& mutex() { 
            return m_lock; 
        };

        DATA* find_nolock(KEY key) {
            return find_(key);
        };

        DATA* find(KEY key) {
            utility::ScopedLock lock(m_lock);
            return find_(key);
        };

        void insert_nolock(KEY key, DATA* data) {
            insert_(key, data);
        };

        void insert(KEY key, DATA* data) {
            utility::ScopedLock lock(m_lock);
            insert_(key, data);
        };

        void remove_nolock(KEY key) {
            remove_(key);
        };

        void remove(KEY key) {
            utility::ScopedLock lock(m_lock);
            remove_(key);
        };

        bool take(KEY key) {
            utility::ScopedLock lock(m_lock);
            return take_(key);
        };

        bool take_nolock(KEY key) {
            return take_(key);
        };

        bool give(KEY key) {
            utility::ScopedLock lock(m_lock);
            return give_(key);
        };

        bool give_nolock(KEY key) {
            return give_(key);
        };

    private:

        typedef std::map<KEY,DATA*> MapType;
        typedef std::set<KEY>       SetType;

        DATA* find_(KEY key) {
            typename MapType::iterator it = m_map.find(key);

            if (m_map.end() == it)
                return NULL;
            else
                return it->second;
        };

        void insert_(KEY key, DATA* data) {
            m_set.insert(key);
            if (m_set.size() == m_depth)
                pop_oldest();

            m_map[key] = data;
        };

        void remove_(KEY key) {
            typename SetType::iterator it = m_set.find(key);
            if (m_set.end() != it)
                m_set.erase(it);
            typename MapType::iterator it2 = m_map.find(key);
            if (m_map.end() != it2) {
                delete it2->second;
                m_map.erase(it2);
            }
        };

        bool take_(KEY key) {

            typename SetType::iterator it = m_set.find(key);
            if (m_set.end() != it) {
                m_set.erase(it);
                return true;
            }

            return false;
        };

        bool give_(KEY key) {

            typename SetType::const_iterator it = m_set.find(key);
            if (m_set.end() == it) {
                m_set.insert(key);
                return true;
            }

            return false;
        };

        void pop_oldest() {

            typename SetType::iterator it = m_set.lower_bound(m_minimum);
            if (m_set.end() != it) {

                typename MapType::iterator it2 = m_map.find(*it);
                if (m_map.end() == it2)
                    CRL_DEBUG("could not find key\n");
                else {
                    delete it2->second;
                    m_map.erase(it2);
                }
                m_set.erase(it);
            }
        };

        const std::size_t m_depth;
        const KEY         m_minimum; // for lower_bound()

        SetType           m_set;
        MapType           m_map;
        utility::Mutex    m_lock;
    };

}}}; // namespaces

#endif // LibMultiSense_details_storage_hh

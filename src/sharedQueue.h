/**
 * \file	serial.h
 * \author	Lucas Mongrain
 * \date	26/10/2017
 * 
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef SHAREDQUEUE_H
#define SHAREDQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SharedQueue
{
public:
    SharedQueue();
    ~SharedQueue();

    T& front();
    void pop_front();

    T get_n_pop_front();

    void push_back(const T& item);
    void push_back(T&& item);


    unsigned long size();
    bool empty();

private:
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

template <typename T>
SharedQueue<T>::SharedQueue(){}

template <typename T>
SharedQueue<T>::~SharedQueue(){}

template <typename T>
T& SharedQueue<T>::front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    return queue_.front();
}

template <typename T>
void SharedQueue<T>::pop_front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    queue_.pop_front();
}

template <typename T>
T SharedQueue<T>::get_n_pop_front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    T temp = queue_.front();
    queue_.pop_front();
    mlock.unlock();
    cond_.notify_one();
    return temp;
}

template <typename T>
void SharedQueue<T>::push_back(const T& item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(item);
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread

}

template <typename T>
void SharedQueue<T>::push_back(T&& item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(std::move(item));
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread

}

template <typename T>
unsigned long SharedQueue<T>::size()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    unsigned long size = queue_.size();
    mlock.unlock();
    cond_.notify_one();
    return size;
}

template <typename T>
bool SharedQueue<T>::empty()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    bool empty = queue_.empty();
    mlock.unlock();
    cond_.notify_one();
    return empty;
}

#endif //SHAREDQUEUE_H

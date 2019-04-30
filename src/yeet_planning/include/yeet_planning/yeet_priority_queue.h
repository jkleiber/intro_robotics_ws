#ifndef YEET_PRIORITY_QUEUE_H
#define YEET_PRIORITY_QUEUE_H

#include <vector>
#include <queue>

//I learned how to do this here: https://stackoverflow.com/questions/19467485/how-to-remove-element-not-at-top-from-priority-queue

template<typename T>
class yeet_priority_queue : public std::priority_queue<T, std::vector<T>, std::greater<T> >
{
    public:
        /**
         * @brief Removes an element from the priority queue
         * 
         * @param val Element to remove
         * @return true If the element was successfully removed
         * @return false If the element could not be removed
         */
        bool remove(const T& val)
        {
            //Find the element in the queue
            auto it = std::find(this->c.begin(), this->c.end(), val);
            
            //If the element was found, remove it and re-sort the queue
            if (it != this->c.end()) 
            {
                this->c.erase(it);
                std::make_heap(this->c.begin(), this->c.end(), this->comp);
                return true;
            }
            
            //The element is not in the queue, so return failure
            return false;
        }


        /**
         * @brief Identifies if an element exists in the queue
         * 
         * @param val The element to find in the queue
         * @return true If the element is in the queue
         * @return false If the element is not in the queue
         */
        bool contains(const T& val)
        {
            //Find the element in the queue
            auto it = std::find(this->c.begin(), this->c.end(), val);

            //Return the status based on the iterator's location
            return (it != this->c.end());
        }


        /**
         * @brief Removes an element from the priority queue
         * 
         * @param val Element to remove
         * @return true If the element was successfully removed
         * @return false If the element could not be removed
         */
        void removeAll(const T& val)
        {
            while(this->contains(val))
            {
                this->remove(val);
            }
        }

};

#endif

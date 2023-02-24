#pragma once

#include <vector>

struct HeapElement
{
    int     uniqueId;
    double  weight;
};

class BinaryHeap
{
    private:
    std::vector<HeapElement> elements;
   
    int parent(int i)
    {
        return (i-1)/2;
    }
    
    int left(int i)
    {
        return (2*i+1);
    }
    
    int right(int i)
    {
        return (2*i+2);
    }
    
    bool helper_idChecker(int givenId){
        int vSize = elements.size();
        int i = 0;
        for(i=0;i<vSize;i++){
            if(givenId == elements[i].uniqueId)
                return true;
        }
        return false;
    }
    void swap(HeapElement &e1,HeapElement &e2){
        HeapElement temp = e1;
        e1 = e2;
        e2 = temp;
    }
    void helper_heapifyDown(int i)
    {
        int size = elements.size();
        int smallest = i;
        int left = 2*i+1;
        int right = 2*i+2;
        if(left<size && elements[left].weight<elements[i].weight){
            smallest = left;
        }
        if(right<size && elements[right].weight<elements[smallest].weight){
            smallest = right;
        }
        if (smallest != i)
        {
        swap(elements[i], elements[smallest]);
        helper_heapifyDown(smallest);
        }
    }
 
    
    void helper_heapify(std::vector<HeapElement>& elements)
    {
        int i = elements.size()-1;
        while(i != 0 && elements[parent(i)].weight>elements[i].weight)
        {
            swap(elements[i],elements[parent(i)]);
            i = parent(i);
        }
    }

    void helper_heapify2(int i){
        int index = i;
        while(index != 0 && elements[parent(index)].weight>elements[index].weight){
            swap(elements[index],elements[parent(index)]);
            index = parent(index);
        }
    }
 
    bool ifExists(int givenId,double newWeight){
        int size = elements.size();
        int i = 0;
        for(i=0;i<size;i++){
            if(givenId == elements[i].uniqueId){
                if(newWeight<elements[i].weight)
                {
                    elements[i].weight = newWeight;
                    helper_heapify2(i);
                    
                }
                else{
                    elements[i].weight = newWeight;
                    helper_heapifyDown(i);
                }
                return true;
            }
        }
        return false;
    }

   
    friend class HW3Tester;

    protected:
    public:
    
        // Constructors & Destructor
                            BinaryHeap();
        //
        bool                Add(int uniqueId, double weight);
        bool                PopHeap(int& outUniqueId, double& outWeight);
        bool                ChangePriority(int uniqueId, double newWeight);
        int                 HeapSize() const;
};

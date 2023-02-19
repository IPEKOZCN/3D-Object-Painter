#include "BinaryHeap.h"

BinaryHeap::BinaryHeap()
{
    // TODO: or not
}


bool BinaryHeap::Add(int uniqueId, double weight)
{
    // TODO:
    if(helper_idChecker(uniqueId) == true)
        return false;
    
    HeapElement temp;
    temp.uniqueId = uniqueId;
    temp.weight = weight;
    elements.push_back(temp);
    helper_heapify(elements);
    return true;
    
}

bool BinaryHeap::PopHeap(int& outUniqueId, double& outWeight)
{
    // TODO:
    if(elements.size() == 0)
        return false;
    if(elements.size() == 1)
        {
        outUniqueId = elements[0].uniqueId;
        outWeight = elements[0].weight; 
        elements.pop_back();
        return true;
        }
    else
    {
        outUniqueId = elements[0].uniqueId;
        outWeight = elements[0].weight; 
        int size = elements.size();
        elements[0] = elements[size-1];
        elements.pop_back();
        helper_heapifyDown(0);
        return true;
    }
        
    
}

bool BinaryHeap::ChangePriority(int uniqueId, double newWeight)
{
    // TODO:
    if(ifExists(uniqueId,newWeight)){
        return true;
    }
    else
        return false;
    
}

int BinaryHeap::HeapSize() const
{
    // TODO:
    return elements.size();
    
}
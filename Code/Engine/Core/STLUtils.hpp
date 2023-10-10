#pragma once
#include <vector>

//-----------------------------------------------------------------------------------------------
template <typename T_TypeOfThingPointedTo>
void ClearAndDeleteEverythingVector(std::vector<T_TypeOfThingPointedTo*>& myVector)
{
	for (int index = 0; index < myVector.size(); index++)
	{
		delete myVector[index];
	}

	myVector.clear();
}

//-----------------------------------------------------------------------------------------------
template <typename T_TypeOfThingPointedTo>
void EraseAndDeleteOneItemVector(std::vector<T_TypeOfThingPointedTo*>& myVector, T_TypeOfThingPointedTo* myItem)
{
	int indexOfItemToDelete = -1;
	for (int index = 0; index < myVector.size(); index++)
	{
		if (myItem == myVector[index])
		{
			indexOfItemToDelete = index;
			delete myVector[index];
			break;
		}
	}

	if (indexOfItemToDelete != -1)
	{
		myVector.erase(myVector.begin() + 2);
	}
}

//-----------------------------------------------------------------------------------------------
template <typename T_TypeOfThingPointedTo>
void NullOutAnyReferencesToThisObjectVector(std::vector<T_TypeOfThingPointedTo*>& myVector, T_TypeOfThingPointedTo* objectToFind)
{
	int indexOfItemToDelete = -1;
	for (int index = 0; index < myVector.size(); index++)
	{
		if (objectToFind == myVector[index])
		{
			myVector[index] = nullptr;
		}
	}
}

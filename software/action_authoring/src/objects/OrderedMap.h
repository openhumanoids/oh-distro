#ifndef ORDERED_MAP_H
#define ORDERED_MAP_H

#include <boost/unordered_map.hpp>
#include <vector>

using namespace boost;
using namespace std;

template <class keyType, class valueType>
class OrderedMap {
private:
	unordered_map<keyType, valueType> keys_to_values;
	vector<keyType> ordered_keys;
public:
	vector<keyType> getOrderedKeys();
	vector<valueType> getOrderedValues();
	valueType& operator[](keyType key);
	bool isKey(keyType key);
	int size();
};

template <class keyType, class valueType>
vector<keyType> OrderedMap<keyType, valueType>::getOrderedKeys() {
	return ordered_keys;
}

template <class keyType, class valueType>
vector<valueType> OrderedMap<keyType, valueType>::getOrderedValues() {
	vector<valueType> ordered_values;
	for (int i = 0; i < (int)ordered_keys.size(); i++)
	{
		ordered_values.push_back(keys_to_values[ordered_keys[i]]);
	}
	return ordered_values;
}

template <class keyType, class valueType>
valueType& OrderedMap<keyType, valueType>::operator[](keyType key) {
	if (keys_to_values.find(key) == keys_to_values.end()){
		ordered_keys.push_back(key);
	}
	return keys_to_values[key];
}

template <class keyType, class valueType>
bool OrderedMap<keyType, valueType>::isKey(keyType key) {
	return keys_to_values.find(key) != keys_to_values.end();
}

template <class keyType, class valueType>
int OrderedMap<keyType, valueType>::size() {
	return (int)keys_to_values.size();
}

#endif //ORDERED_MAP_H
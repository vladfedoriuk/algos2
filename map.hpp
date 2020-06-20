//
// Created by Vlad_F on 06.04.2020.
//

#ifndef ALGOS2_MAP_HPP
#define ALGOS2_MAP_HPP

#include <cstdint>
#include <utility>
#include <iterator>
#include "bst.hpp"
#include<exception>
#include<functional>

// Uwaga! Kod powinien być odporny na błędy i każda z metod jeżeli zachodzi niebezpieczeństwo wywołania z niepoprawnymi parametrami powinna zgłaszać odpowiednie wyjątki!

// klasa reprezentująca słownik (map) oparty na drzewie BST/AVL
// K - typ kluczy
// V - typ wartości
template <typename K, typename V>
class Map
{
private:
    class Pair{
    public:
        K key;
        mutable V value;
        Pair();
        Pair(const K& key, const V& value);
        Pair(const Pair& p);
        Pair(Pair&& p);
        Pair& operator=(const Pair& p);
        Pair& operator=(Pair&& p);
        bool operator>(const Pair &p) const;
        bool operator<(const Pair &p) const;
        bool operator==(const Pair &p) const;
        bool operator!=(const Pair &p) const;
        bool operator>=(const Pair &p) const;
        bool operator<=(const Pair& p) const;
    };
public:
    class Iterator
    {
// ...
    private:
        typename BST<typename Map<K, V>::Pair>::iterator iter;
    public:
        Iterator();
        Iterator(const typename BST<Pair>::iterator& it);
        const typename BST<Pair>::iterator& get_it() const;
        bool operator==(const Iterator &it) const;
        bool operator!=(const Iterator &it) const;
        Iterator& operator++();
        Iterator operator++(int);
        V& operator*() const;
        V* operator->() const;
        operator bool() const;
    };

    class ConstIterator
    {
// ...
    private:
        typename BST<Pair>::iterator iter;

    public:
        ConstIterator();
        ConstIterator(const typename BST<typename Map<K, V>::Pair>::iterator& it);
        const typename BST<Pair>::iterator& get_it() const;
        bool operator==(const ConstIterator &it) const;
        bool operator!=(const ConstIterator &it) const;
        ConstIterator& operator++();
        ConstIterator operator++(int);
        const V& operator*() const;
        const V* operator->() const;
        operator bool() const;
    };

// ...
private:
    BST<Pair> bst_;
public:
    const BST<Pair>& get_tree() const{
        return bst_;
    };
    Map();
    Map(const Map<K, V> &source);
    Map<K, V>& operator=(const Map<K, V> &source);
    Map(Map<K, V> &&source);
    Map<K, V>& operator=(Map<K, V> &&source);
    ~Map();

// zwraca ilość elementów
    std::size_t size() const;
// dodaje klucz i wartość - zwraca "Iterator" do dodanej wartości i true, lub "Iterator" do istniejącej wartości i false, jeżeli z jakiegoś powodu nie udało się dodać/znaleźć to zwraca false i to samo co end()
    std::pair<Iterator, bool> insert(const std::pair<K, V> &key_value);
// wyszukuje element o podanym kluczu - jeżeli element został znaleziony to zwraca "ConstIterator" do znalezionej wartości, jeżeli nie to zwraca to samo co end()
    ConstIterator find(const K &key) const;
// wyszukuje element o podanym kluczu - jeżeli element został znaleziony to zwraca "Iterator" do znalezionej wartości, jeżeli nie to zwraca to samo co end()
    Iterator find(const K &key);
// wyszukuje element o podanym kluczu - jeżeli element został znaleziony to zwraca referencję do znalezionej (stałej) wartości, w innym przypadku zgłasza wyjątek
    const V& operator[](const K &key) const;
// wyszukuje element o podanym kluczu - jeżeli element został znaleziony to zwraca referencję do znalezionej wartości, jeżeli nie to dodaje nowy element o podanym kluczu i domyślnej wartości V() i zwraca referencję do wartości
    V& operator[](const K &key);
// usuwa element o podanej wartości - jeżeli element został usunięty to zwraca "Iterator" na kolejny element, jeżeli elementu o podanej wartości nie udało się odnaleźć to zwraca to samo co "end()"
    Iterator remove_it(const K &key);
    Iterator remove(const K &key);
    //bool remove(const K& key);
// usuwa wszystkie elementy
    void clear();

// zwraca "ConstIterator" na pierwszy element
    ConstIterator begin() const;
// zwraca "Iterator" na pierwszy element
    Iterator begin();
// zwraca "ConstIterator" "za ostatni" element
    ConstIterator end() const;
// zwraca "Iterator" "za ostatni" element
    Iterator end();
};

template<typename K, typename V>
bool Map<K, V>::ConstIterator::operator==(const Map::ConstIterator &it) const {
    return iter == it.get_it();
}

template<typename K, typename V>
const typename BST<typename Map<K, V>::Pair>::iterator &Map<K, V>::ConstIterator::get_it() const {
    return iter;
}

template<typename K, typename V>
Map<K, V>::ConstIterator::ConstIterator(const typename BST<typename Map<K, V>::Pair>::iterator &it): iter(it) {

}

template<typename K, typename V>
bool Map<K, V>::ConstIterator::operator!=(const Map::ConstIterator &it) const {
    return !(iter == it.get_it());
}

template<typename K, typename V>
typename Map<K, V>::ConstIterator &Map<K, V>::ConstIterator::operator++() {
    ++iter;
    return *this;
}

template<typename K, typename V>
typename Map<K, V>::ConstIterator Map<K, V>::ConstIterator::operator++(int) {
    return Map::ConstIterator(typename BST<typename Map<K, V>::Pair>::iterator(iter++));
}

template<typename K, typename V>
const V& Map<K, V>::ConstIterator::operator*() const {
    return (*iter).value;
}

template<typename K, typename V>
const V* Map<K, V>::ConstIterator::operator->() const {
    return &((*iter).value);
}

template<typename K, typename V>
Map<K, V>::ConstIterator::operator bool() const {
    return iter.ptr != nullptr || false;
}

template<typename K, typename V>
Map<K, V>::ConstIterator::ConstIterator(): iter(typename BST<typename Map<K, V>::Pair>::iterator())  {

}

template<typename K, typename V>
Map<K, V>::Iterator::Iterator(const typename BST<Pair>::iterator &it): iter(it) {

}

template<typename K, typename V>
const typename BST<typename Map<K, V>::Pair>::iterator& Map<K, V>::Iterator::get_it() const {
    return iter;
}

template<typename K, typename V>
bool Map<K, V>::Iterator::operator==(const Map::Iterator &it) const {
    return iter == it.get_it();
}

template<typename K, typename V>
bool Map<K, V>::Iterator::operator!=(const Map::Iterator &it) const {
    return !(iter==it.get_it());
}

template<typename K, typename V>
typename Map<K, V>::Iterator &Map<K, V>::Iterator::operator++() {
    ++iter;
    return *this;
}

template<typename K, typename V>
typename Map<K, V>::Iterator Map<K, V>::Iterator::operator++(int) {
    return Map::Iterator(typename BST<typename Map<K, V>::Pair>::iterator(iter++));
}

template<typename K, typename V>
V &Map<K, V>::Iterator::operator*() const {
    return (*iter).value;
}

template<typename K, typename V>
V *Map<K, V>::Iterator::operator->() const {
    return &((*iter).value);
}

template<typename K, typename V>
Map<K, V>::Iterator::operator bool() const {
    return iter.ptr != nullptr || false;
}

template<typename K, typename V>
Map<K, V>::Iterator::Iterator() :iter(typename BST<typename Map<K, V>::Pair>::iterator()) {

}

template<typename K, typename V>
Map<K, V>::Map() {

}

template<typename K, typename V>
Map<K, V>::Map(const Map<K, V>& source) {
    bst_ = source.get_tree();
}

template<typename K, typename V>
Map<K, V>& Map<K, V>::operator=(const Map<K, V> &source) {
    bst_ = source.get_tree();
    return *this;
}

template<typename K, typename V>
Map<K, V>::Map(Map<K, V> &&source) {
    bst_ = std::move(source.get_tree());
}

template<typename K, typename V>
Map<K, V> &Map<K, V>::operator=(Map<K, V> &&source) {
    bst_ = std::move(source.get_tree());
    return *this;
}

template<typename K, typename V>
Map<K, V>::~Map() {

}

template<typename K, typename V>
std::size_t Map<K, V>::size() const {
    return bst_.size();
}

// dodaje klucz i wartość - zwraca "Iterator" do dodanej wartości i true, lub "Iterator" do istniejącej wartości i false, jeżeli z jakiegoś powodu nie udało się dodać/znaleźć to zwraca false i to samo co end()
template<typename K, typename V>
std::pair<typename Map<K, V>::Iterator, bool> Map<K, V>::insert(const std::pair<K, V> &key_value) {
    Map<K, V>::Pair p(key_value.first, key_value.second);
    bool flag = bst_.insert(p);
    typename BST<typename Map<K, V>::Pair>::iterator it;
    for (it = bst_.begin(); it != bst_.end(); ++it) {
        if (*it == p) break;
    }
    if(it.ptr) return std::make_pair(typename Map<K, V>::Iterator(it), flag);
    return std::make_pair(typename Map<K, V>::Iterator(bst_.end()), false);
}

template<typename K, typename V>
typename Map<K, V>::ConstIterator Map<K, V>::find(const K &key) const {
    typename BST<typename Map<K, V>::Pair>::iterator it;
    for(it = bst_.begin(); it!=bst_.end(); ++it){
        if ( it.ptr && (*it).key == key) break;
    }
    if (it.ptr) return Map<K, V>::ConstIterator(it);
    return end();
}

template<typename K, typename V>
typename Map<K, V>::Iterator Map<K, V>::find(const K &key) {
    typename BST<typename Map<K, V>::Pair>::iterator it;
    for(it = bst_.begin(); it!=bst_.end(); ++it){
        if ( it.ptr && (*it).key == key) break;
    }
    if (it.ptr) return Map<K, V>::Iterator(it);
    return end();
}

template<typename K, typename V>
const V &Map<K, V>::operator[](const K &key) const {
    typename BST<typename Map<K, V>::Pair>::iterator it;
    for(it = bst_.begin(); it!=bst_.end(); ++it){
        if ( it.ptr && (*it).key == key) break;
    }
    if (it.ptr) return (*it).value;
    throw std::runtime_error("such a key does not exist in the map");
}

// wyszukuje element o podanym kluczu - jeżeli element został znaleziony to zwraca referencję do znalezionej wartości,
// jeżeli nie to dodaje nowy element o podanym kluczu i domyślnej wartości V() i zwraca referencję do wartości

template<typename K, typename V>
V &Map<K, V>::operator[](const K &key) {
    Map<K, V>::Pair p(key, V());
    typename BST<typename Map<K, V>::Pair>::node_ptr  ptr = bst_.search(p);
    if(ptr) return ((ptr->data).value);
    bst_.insert(p);
    return (bst_.search(p)->data).value;

}

// usuwa element o podanej wartości - jeżeli element został usunięty to zwraca "Iterator" na kolejny element, jeżeli
// elementu o podanej wartości nie udało się odnaleźć to zwraca to samo co "end()"

template<typename K, typename V>
typename Map<K, V>::Iterator Map<K, V>::remove_it(const K &key) {
    //O(log n)
    Map<K, V>::Pair p(key, V());
    typename BST<typename Map<K, V>::Pair>::node_ptr ptr = bst_.search(p);
    if(!ptr) return end();
    auto ptr1 = BST<typename Map<K, V>::Pair>::find_succ(ptr);
    if(!ptr1){
        bst_.delete_node(ptr);
        return end();
    }
    auto data = ptr1->data;
    bst_.delete_node(ptr);
    return Iterator(typename BST<typename Map<K, V>::Pair>::iterator(bst_.search(data)));
}

template<typename K, typename V>
void Map<K, V>::clear() {
   bst_.clear();
}

template<typename K, typename V>
typename Map<K, V>::ConstIterator Map<K, V>::begin() const {
    return Map<K, V>::ConstIterator(bst_.begin());
}

template<typename K, typename V>
typename Map<K,V>::Iterator Map<K, V>::begin() {
    return Map<K, V>::Iterator(bst_.begin());
}

template<typename K, typename V>
typename Map<K, V>::ConstIterator Map<K, V>::end() const {
    return Map<K, V>::ConstIterator(bst_.end());
}

template<typename K, typename V>
typename Map<K, V>::Iterator Map<K, V>::end() {
    return Map<K, V>::Iterator(bst_.end());
}

template<typename K, typename V>
typename Map<K, V>::Iterator Map<K, V>::remove(const K &key) {
    return remove_it(key);
}

/*
template<typename K, typename V>
bool Map<K, V>::remove(const K& key) {
    return bool(remove_it(key));
}*/

template<typename K, typename V>
Map<K, V>::Pair::Pair(const K &key, const V &value): key(key), value(value) {

}

template<typename K, typename V>
Map<K, V>::Pair::Pair(const Map::Pair &p): value(p.value), key(p.key) {

}

template<typename K, typename V>
Map<K, V>::Pair::Pair(Map::Pair &&p): key(std::move(p.key)), value(std::move(p.value)){

}

template<typename K, typename V>
typename Map<K, V>::Pair &Map<K, V>::Pair::operator=(const Map::Pair &p) {
    key = p.key;
    value = p.value;
    return *this;
}

template<typename K, typename V>
typename Map<K, V>::Pair &Map<K, V>::Pair::operator=(Map::Pair &&p) {
    key = std::move(p.key);
    value = std::move(p.value);
    return *this;
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator==(const Map::Pair &p) const{
    return key==p.key;
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator>(const Map::Pair &p) const{
    return key > p.key;
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator>=(const Map::Pair &p) const{
    return *this > p || *this == p;
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator<=(const Map::Pair &p) const{
    return *this < p || *this == p;
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator!=(const Map::Pair &p) const{
    return !(*this==p);
}

template<typename K, typename V>
bool Map<K, V>::Pair::operator<(const Map::Pair &p) const {
    return key < p.key;
}

template<typename K, typename V>
Map<K, V>::Pair::Pair(): key(K()), value(V()) {

}

#endif //ALGOS2_MAP_HPP

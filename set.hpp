#pragma once
#include "bst.hpp"
#include<utility>
#include<iterator>

template<typename T>
class Set
{
public:
	Set();
	Set(const Set<T>& source);
	Set<T>& operator=(const Set<T>& source);
	Set(Set<T>&& source);
	Set<T>& operator=(Set<T>&& source);
	~Set();
private:
	BST<T> bst_;
public:
	class Iterator : public std::iterator<std::input_iterator_tag, const T>
	{
		typename BST<T>::iterator iter;
	public:
	    Iterator(const typename BST<T>::iterator& it);
		const typename BST<T>::iterator& get_it() const;
		bool operator==(const typename Set<T>::Iterator &it) const;
		bool operator!=(const typename Set<T>::Iterator &it) const { return !(*this == it); }
		Iterator& operator++();
		Iterator operator++(int);
		const T& operator*() const;
		const T* operator->() const;
		operator bool() const;
	};
	const BST<T>& into_bst() const {
		return bst_;
	};
	std::size_t size() const;
	bool empty() const;
	std::pair<typename Set<T>::Iterator, bool> insert(const T& value);
	typename Set<T>::Iterator find(const T &value) const;
	typename Set<T>::Iterator begin() const;
	typename Set<T>::Iterator end() const;
	//const T* insert(const T& value);
	//const T* find(const T& value) const;
	bool remove(const T& value);
	void inorder(std::function<void(const T&)> f) const;

};

template<typename T>
inline Set<T>::Set()
{

}
template<typename T>
inline Set<T>::Set(const Set<T>& source) :bst_(source.into_bst())
{

};

template<typename T>
inline Set<T>& Set<T>::operator=(const Set<T>& source)
{
	bst_ = source.into_bst();
	return *this;
}
template<typename T>
inline Set<T>::Set(Set<T>&& source): bst_(std::move(source.into_bst()))
{
}
template<typename T>
inline Set<T>& Set<T>::operator=(Set<T>&& source)
{
	bst_ = std::move(source.into_bst());
	return *this;
}
template<typename T>
inline Set<T>::~Set()
{

};

template<typename T>
inline std::size_t Set<T>::size() const
{
	return bst_.size();
};

template<typename T>
inline bool Set<T>::empty() const
{
	return (bst_.size() == 0);
};

/*
template<typename T>
inline const T * Set<T>::insert(const T & value)
{
	if (bst_.insert(value)) {
		return &value;
	}
	else {
		return &(bst_.search(value)->data);
	}
	
};
*/

/*template<typename T>
inline const T * Set<T>::find(const T & value) const
{
	return &(bst_.search(value)->data);
};*/

template<typename T>
inline typename Set<T>::Iterator Set<T>::begin() const
{
	return typename Set<T>::Iterator(bst_.begin());
};

template<typename T>
inline typename Set<T>::Iterator Set<T>::end() const
{
	return typename Set<T>::Iterator(bst_.end());
};

template<typename T>
inline bool Set<T>::remove(const T & value)
{
	typename BST<T>::node_ptr p = bst_.search(value);
	if (p) {
		//bst_.BSTremove(p);
		bst_.delete_node(p);
		return true;
	}
	else {
		return false;
	}
};

template<typename T>
inline void Set<T>::inorder(std::function<void(const T&)> f) const
{
	bst_.traverse_in_order(f);
};

template<typename T>
inline Set<T>::Iterator::Iterator(const typename BST<T>::iterator & it)
{
	iter = it;
};

template<typename T>
inline const typename BST<T>::iterator & Set<T>::Iterator::get_it() const
{
	return iter;
};

template<typename T>
inline bool Set<T>::Iterator::operator==(const typename Set<T>::Iterator & it) const
{
	return iter == it.get_it();
};

template<typename T>
inline typename Set<T>::Iterator & Set<T>::Iterator::operator++()
{
	++iter;
	return *this;
};

template<typename T>
inline typename Set<T>::Iterator Set<T>::Iterator::operator++(int)
{
	return typename Set<T>::Iterator(iter++);
};

template<typename T>
inline const T & Set<T>::Iterator::operator*() const
{
	return (iter.ptr)->data;
};

template<typename T>
inline const T * Set<T>::Iterator::operator->() const
{
	return &(iter.ptr->data);
};

template<typename T>
inline Set<T>::Iterator::operator bool() const
{
    return iter.ptr != nullptr || false;
};

template<typename T>
std::pair<typename Set<T>::Iterator, bool> Set<T>::insert(const T &value) {
	bool flag = bst_.insert(value);
	typename BST<T>::iterator it;
	for (it = bst_.begin(); it != bst_.end(); ++it) {
	  if (*it == value) break;
	}
	if(it.ptr) return std::make_pair(typename Set<T>::Iterator(it), flag);
	return std::make_pair(typename Set<T>::Iterator(bst_.end()), false);
};

template<typename T>
typename Set<T>::Iterator Set<T>::find(const T &value) const {
	typename BST<T>::node_ptr ptr = bst_.search(value);
	if (ptr) return Set::Iterator(typename BST<T>::iterator(ptr));
	else {
		return Set<T>::Iterator(bst_.end());
	}
};

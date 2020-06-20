//
// Created by Vlad_F on 30.03.2020.
//

#ifndef ALGOS2_UNIVERSALSET_HPP
#define ALGOS2_UNIVERSALSET_HPP

#include <utility>
#include <iterator>
#include <cstdint>
#include <array>
#include <bitset>
#include<exception>

template<typename T, std::size_t UNIVERSE_SIZE>
class UniversalSet{
public:
    class Vector{
    public:
        class Iterator : public std::iterator<std::input_iterator_tag, const T>
        {
        public:
            explicit Iterator(const Vector* v);
            Iterator(const Vector* v, const size_t& i);
            Iterator(const Iterator& it);
            Iterator& operator=(const Iterator& it);
            Iterator(Iterator&& it);
            Iterator& operator=(Iterator&& it);

            friend class Vector;

        private:
            std::size_t index;
            const Vector* vector;
        public:
            const Vector* get_v() const{
                return vector;
            };
            [[nodiscard]] const std::size_t& get_index() const{
                return index;
            };
            bool operator==(const Iterator& it) const;
            bool operator!=(const Iterator& it) const;
            Iterator& operator++();
            Iterator operator++(int);
            const T& operator*() const;
            const T* operator->() const;
            operator bool() const;
        };

    public:
        explicit Vector(const std::bitset<UNIVERSE_SIZE>& bts, UniversalSet<T, UNIVERSE_SIZE>* un);
        Vector(const Vector& v);
        Vector& operator=(const Vector& v);
        Vector(Vector&& v);
        Vector& operator=(Vector&& v);

        friend class Iterator;

    private:
        std::bitset<UNIVERSE_SIZE> vec;
        UniversalSet<T, UNIVERSE_SIZE>* universe;
    public:
        const std::bitset<UNIVERSE_SIZE>& into_bts() const{
            return vec;
        };
        UniversalSet<T, UNIVERSE_SIZE>* get_universe() const{
            return universe;
        }
        std::size_t count() const;
        std::pair<Iterator, bool> insert(std::size_t i);
        bool isMember(std::size_t i) const;
        Iterator elemIterator(std::size_t i) const;
        bool remove(std::size_t i);
        bool operator==(const Vector& v2) const;
        bool operator!=(const Vector& v2) const;
        Vector operator+(const Vector& v2) const;
        Vector operator-(const Vector& v2) const;
        Vector operator*(const Vector& v2) const;
        Vector operator/(const Vector& v2) const;
        Iterator begin() const;
        Iterator end() const;
    };

public:
    explicit UniversalSet(const std::array<T, UNIVERSE_SIZE>& elems);
    UniversalSet(const UniversalSet& set);
    UniversalSet& operator=(const UniversalSet& set);
    UniversalSet(UniversalSet&& set)  noexcept;
    UniversalSet& operator=(UniversalSet&& set);

private:
    const std::array<T, UNIVERSE_SIZE> elems_;

public:
    const std::array<T, UNIVERSE_SIZE>& into_arr() const{
        return elems_;
    };
    constexpr std::size_t universeSize() const { return UNIVERSE_SIZE; }
    const T& elem(std::size_t i) const;
    const T& operator[](std::size_t i) const;
    Vector makeVector() const;
};

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::Iterator(const UniversalSet::Vector *v): vector(v), index(0){
   while(index < UNIVERSE_SIZE && !vector->vec.test(index)) ++index;
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::Iterator(const UniversalSet::Vector *v, const size_t &i): vector(v), index(i) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::Iterator(const UniversalSet::Vector::Iterator &it): vector(it.get_v()), index(it.get_index()) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator &
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator=(const UniversalSet::Vector::Iterator &it) {
    vector = it.get_v();
    index = it.get_index();
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::Iterator(UniversalSet::Vector::Iterator &&it): vector(std::move(it.get_v())), index(std::move(it.get_index())) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator &
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator=(UniversalSet::Vector::Iterator &&it) {
    vector = std::move(it.get_v());
    index  = std::move(it.get_index());
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator==(const UniversalSet::Vector::Iterator &it) const {
    return (vector == it.get_v() && index == it.get_index());
}

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator!=(const UniversalSet::Vector::Iterator &it) const {
    return !(*this == it);
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator& UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator++() {
    ++index;
    while(index < UNIVERSE_SIZE && !(vector->vec.test(index))) ++index;
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator++(int) {
    auto index1 = index;
    ++index;
    while( index < UNIVERSE_SIZE && !vector->vec.test(index)) ++index;
    return UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator(vector, index1);
}

template<typename T, std::size_t UNIVERSE_SIZE>
const T &UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator*() const {
    return vector->universe->into_arr()[index];
}

template<typename T, std::size_t UNIVERSE_SIZE>
const T *UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator->() const {
    return &(vector->universe[index]);
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator::operator bool() const {
    return index < UNIVERSE_SIZE && vector->vec[index];
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::UniversalSet(const std::array<T, UNIVERSE_SIZE> &elems): elems_(elems) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::UniversalSet(const UniversalSet & set): elems_(set.into_arr()) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE> &UniversalSet<T, UNIVERSE_SIZE>::operator=(const UniversalSet & set) {
    elems_ = set.into_arr();
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::UniversalSet(UniversalSet && set) noexcept: elems_(std::move(set.into_arr())) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE> &UniversalSet<T, UNIVERSE_SIZE>::operator=(UniversalSet && set) {
    elems_ = std::move(set.into_arr());
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
const T& UniversalSet<T, UNIVERSE_SIZE>::elem(std::size_t i) const {
    return elems_.at(i);
}

template<typename T, std::size_t UNIVERSE_SIZE>
const T &UniversalSet<T, UNIVERSE_SIZE>::operator[](std::size_t i) const {
    if ( i < 0 or i >= UNIVERSE_SIZE) throw std::runtime_error("index out of range\n");
    else
        return elems_[i];
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector UniversalSet<T, UNIVERSE_SIZE>::makeVector() const {
    return UniversalSet::Vector(std::bitset<UNIVERSE_SIZE>(), const_cast<UniversalSet<T, UNIVERSE_SIZE>*>(this));
}

template<typename T, std::size_t UNIVERSE_SIZE>
std::size_t UniversalSet<T, UNIVERSE_SIZE>::Vector::count() const {
    return vec.count();
}

template<typename T, std::size_t UNIVERSE_SIZE>
std::pair<typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator, bool> UniversalSet<T, UNIVERSE_SIZE>::Vector::insert(std::size_t i) {
    if ( i < 0 || i >= UNIVERSE_SIZE) throw std::runtime_error("index out o range\n");
    if (vec.test(i)){
        return std::make_pair(end(), false);
    } else{
        vec.set(i);
        return std::make_pair(typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator(this, i), true);
    }
};

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::isMember(std::size_t i) const {
    if (i < UNIVERSE_SIZE && i >=0) {
        return vec.test(i);
    } else
        throw std::runtime_error("index is out of range\n");
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator UniversalSet<T, UNIVERSE_SIZE>::Vector::elemIterator(std::size_t i) const {
    if ( i < 0 || i >= UNIVERSE_SIZE) throw std::runtime_error("index out of range\n");
    if (vec.test(i)){
        return typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator(this, i);
    } else{
        return end();
    }
}

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::remove(std::size_t i) {
    if (vec.test(i)){
        vec.reset(i);
        return true;
    } else{
        return false;
    }
}

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::operator==(const UniversalSet::Vector &v2) const {
    return (v2.get_universe()->into_arr() == universe->into_arr() && v2.into_bts() == vec);
}

template<typename T, std::size_t UNIVERSE_SIZE>
bool UniversalSet<T, UNIVERSE_SIZE>::Vector::operator!=(const UniversalSet::Vector &v2) const {
    return !(*this==v2);
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector UniversalSet<T, UNIVERSE_SIZE>::Vector::operator+(const UniversalSet::Vector &v2) const {
    if (universe->into_arr() != v2.get_universe()->into_arr()) throw std::runtime_error("wrong universe\n");
    return UniversalSet::Vector(vec | v2.into_bts(),  universe);
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Vector(const std::bitset<UNIVERSE_SIZE>& bts, UniversalSet<T, UNIVERSE_SIZE>* un): vec(bts), universe(un){

}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector UniversalSet<T, UNIVERSE_SIZE>::Vector::operator-(const UniversalSet::Vector &v2) const {
    if (universe->into_arr() != v2.get_universe()->into_arr()) throw std::runtime_error("wrong universe\n");
    return UniversalSet::Vector((vec ^ v2.into_bts()) & vec, universe);
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector UniversalSet<T, UNIVERSE_SIZE>::Vector::operator*(const UniversalSet::Vector &v2) const {
    if (universe->into_arr() != v2.get_universe()->into_arr()) throw std::runtime_error("wrong universe\n");
    return UniversalSet::Vector(vec & v2.into_bts(), universe);
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector UniversalSet<T, UNIVERSE_SIZE>::Vector::operator/(const UniversalSet::Vector &v2) const {
    if (universe->into_arr() != v2.get_universe()->into_arr()) throw std::runtime_error("wrong universe\n");
    return UniversalSet::Vector(vec ^ v2.into_bts(), universe);
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Vector(const UniversalSet::Vector& v): vec(v.into_bts()), universe(v.get_universe()) {

}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector &UniversalSet<T, UNIVERSE_SIZE>::Vector::operator=(const UniversalSet::Vector & v) {
    vec = v.into_bts();
    universe = v.get_universe();
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
UniversalSet<T, UNIVERSE_SIZE>::Vector::Vector(UniversalSet::Vector && v): vec(std::move(v.into_bts())), universe(std::move(v.get_universe())){

}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector &UniversalSet<T, UNIVERSE_SIZE>::Vector::operator=(UniversalSet::Vector &&v) {
    vec = std::move(v.into_bts());
    universe = std::move(v.get_universe());
    return *this;
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename  UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator UniversalSet<T, UNIVERSE_SIZE>::Vector::begin() const {
    return UniversalSet::Vector::Iterator(this);
}

template<typename T, std::size_t UNIVERSE_SIZE>
typename UniversalSet<T, UNIVERSE_SIZE>::Vector::Iterator UniversalSet<T, UNIVERSE_SIZE>::Vector::end() const {
    return UniversalSet::Vector::Iterator(this, UNIVERSE_SIZE);
}



#endif

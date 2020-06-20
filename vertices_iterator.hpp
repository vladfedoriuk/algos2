//
// Created by Vlad_F on 19.05.2020.
//

#ifndef ALGOS2_VERTICES_ITERATOR_HPP
#define ALGOS2_VERTICES_ITERATOR_HPP

#include<type_traits>
#include<iterator>

template<bool AmConst, typename V>
class VIterator: public std::iterator<std::input_iterator_tag, V>{
public:
    using vertices_t = typename std::conditional_t<AmConst, const std::vector<V>*, std::vector<V>*>;
    using reference = typename std::conditional_t<AmConst, const V&, V&>;
    using pointer = typename std::conditional_t<AmConst, const V*, V*>;
private:
    vertices_t vertices;
    std::size_t index;
public:
    VIterator(const VIterator&) = default;
    VIterator& operator=(const VIterator&) = default;

    template<bool IsConst, class = std::enable_if_t<AmConst && !IsConst>>
    VIterator(const VIterator<IsConst, V>& vit): vertices(reinterpret_cast<vertices_t>(vit.get_vertices())),
    index(vit.id()){

    };

    template<bool IsConst, class = std::enable_if_t<AmConst && !IsConst>>
    VIterator& operator=(const VIterator<IsConst, V>& vit){
        vertices = reinterpret_cast<vertices_t>(vit.get_vertices());
        index = vit.id();
    };

    vertices_t get_vertices() const;
    VIterator(vertices_t vertices, const std::size_t& index);
    bool operator==(const VIterator &vi2) const;
    bool operator!=(const VIterator &vi2) const;
    VIterator& operator++();
    VIterator operator++(int);
    reference operator*() const;
    pointer operator->() const;
    std::size_t id() const;
    operator bool() const;
};

template<bool AmConst, typename V>
VIterator<AmConst, V> VIterator<AmConst, V>::operator++(int) {
    return VIterator(vertices, index++);
}

template<bool AmConst, typename V>
bool VIterator<AmConst, V>::operator!=(const VIterator &vi2) const {
    return !(*this==vi2);
}

template<bool AmConst, typename V>
VIterator<AmConst, V>::VIterator(vertices_t vertices, const std::size_t &index): vertices(vertices), index(index) {

}

template<bool AmConst, typename V>
typename VIterator<AmConst, V>::vertices_t VIterator<AmConst, V>::get_vertices() const {
    return vertices;
}

template<bool AmConst, typename V>
bool VIterator<AmConst, V>::operator==(const VIterator &vi2) const {
    return  vertices == vi2.get_vertices() && index == vi2.id();
}

template<bool AmConst, typename V>
VIterator<AmConst, V>& VIterator<AmConst, V>::operator++() {
    ++index;
    return *this;
}

template<bool AmConst, typename V>
typename VIterator<AmConst, V>::reference VIterator<AmConst, V>::operator*() const {
    (vertices)->at(index);
}

template<bool AmConst, typename V>
typename VIterator<AmConst, V>::pointer VIterator<AmConst, V>::operator->() const {
    return &(vertices)->at(index);
}

template<bool AmConst, typename V>
std::size_t VIterator<AmConst, V>::id() const {
    return index;
}

template<bool AmConst, typename V>
VIterator<AmConst, V>::operator bool() const {
    return vertices && index >= 0 && index < vertices->size();
}


#endif //ALGOS2_VERTICES_ITERATOR_HPP

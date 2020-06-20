//
// Created by Vlad_F on 19.05.2020.
//

#ifndef ALGOS2_EDGES_ITERATOR_HPP
#define ALGOS2_EDGES_ITERATOR_HPP
#include"graph.hpp"
#include<type_traits>
template<bool AmConst, typename E>
class EIterator: public std::iterator<std::input_iterator_tag, E>{
public:
    using matrix_t = typename std::conditional_t<AmConst, const std::vector<std::vector<std::optional<E>>>*,
            std::vector<std::vector<std::optional<E>>>*>;
    using reference = typename std::conditional_t<AmConst, const E&, E&>;
    using pointer = typename std::conditional_t<AmConst, const E*, E*>;
private:
    matrix_t matrix;
    std::size_t from;
    std::size_t to;
public:
    EIterator(const EIterator&) = default;
    EIterator& operator=(const EIterator&) = default;

    template<bool IsConst, class = std::enable_if_t<AmConst && !IsConst>>
    EIterator(const EIterator<IsConst, E>& eit): matrix(reinterpret_cast<matrix_t>(eit.get_matrix())),
                                                 from(eit.v1id()), to(eit.v2id()){

    };

    template<bool IsConst, class = std::enable_if_t<AmConst && !IsConst>>
    EIterator& operator=(const EIterator<IsConst, E>& eit){
        matrix = reinterpret_cast<matrix_t>(eit.get_matrix());
        from= eit.v1id();
        to = eit.v2id();
    };

    matrix_t get_matrix() const;
    EIterator(matrix_t matrix, std::size_t from, std::size_t to=0);
    bool operator==(const EIterator &ei) const;
    bool operator!=(const EIterator &ei) const;
    EIterator& operator++();
    EIterator operator++(int);
    reference operator*() const;
    pointer operator->() const;
    std::size_t v1id() const;
    std::size_t v2id() const;
    operator bool() const;

};

template<bool AmConst, typename E>
typename EIterator<AmConst, E>::matrix_t EIterator<AmConst, E>::get_matrix() const {
    return matrix;
}

template<bool AmConst, typename E>
EIterator<AmConst, E>::EIterator(matrix_t matrix, std::size_t from,
                                 std::size_t to) : matrix(matrix), from(from), to(to) {
    std::size_t i=this->from;
    std::size_t j=this->to;
    for(;i<matrix->size(); i++){
        for(;j<matrix->size(); j++){
            if ((*matrix)[i][j].has_value()) break;
        }
        if (j<matrix->size()) break;
        j=0;
    }

    this->from = i;
    this->to = j;

}

template<bool AmConst, typename E>
bool EIterator<AmConst, E>::operator==(const EIterator &ei) const {
    return from == ei.v1id() && to==ei.v2id() && matrix == ei.get_matrix();
}

template<bool AmConst, typename E>
bool EIterator<AmConst, E>::operator!=(const EIterator &ei) const {
    return !(*this==ei);
}

template<bool AmConst, typename E>
EIterator<AmConst, E>& EIterator<AmConst, E>::operator++() {
    std::size_t i;
    std::size_t j;
    if(to+1<matrix->size()){
        i = from;
        j = to+1;
    } else{
        i = from+1;
        j=0;
    }
    for(;i<matrix->size();i++){
        for(;j<matrix->size(); j++){
            if ((*matrix)[i][j].has_value()) break;
        }
        if (j<matrix->size()) break;
        j=0;
    }

    from = i;
    to = j;
    return *this;
}

template<bool AmConst, typename E>
EIterator<AmConst, E> EIterator<AmConst, E>::operator++(int) {
    std::size_t i;
    std::size_t j;
    if(to+1<matrix->size()){
        i = from;
        j = to+1;
    } else{
        i = from+1;
        j=0;
    }
    for(;i<matrix->size();i++){
        for(;j<matrix->size(); j++){
            if ((*matrix)[i][j].has_value()) break;
        }
        if (j<matrix->size()) break;
        j=0;
    }

    std::size_t temp_from = from;
    std::size_t temp_to = to;
    from = i;
    to = j;
    return EIterator(matrix, temp_from, temp_to);
}

template<bool AmConst, typename E>
typename EIterator<AmConst, E>::reference EIterator<AmConst, E>::operator*() const {
    return (*matrix)[from][to].value();
}

template<bool AmConst, typename E>
typename EIterator<AmConst, E>::pointer EIterator<AmConst, E>::operator->() const {
    return &((*matrix)[from][to].value());
}

template<bool AmConst, typename E>
std::size_t EIterator<AmConst, E>::v1id() const {
    return from;
}

template<bool AmConst, typename E>
std::size_t EIterator<AmConst, E>::v2id() const {
    return to;
}

template<bool AmConst, typename E>
EIterator<AmConst, E>::operator bool() const {
    return matrix && from >= 0 && to >=0 && from < matrix->size() && to < matrix->size();
}


#endif //ALGOS2_EDGES_ITERATOR_HPP

#pragma once
#include <memory>

/** @file abstract_type.hpp 依赖：无（HAL 根） */
class AbstractType {
public:
    virtual ~AbstractType() = default;
    virtual std::unique_ptr<AbstractType> clone() const = 0;
};

#ifndef COMPARISON_NODE_HPP
#define COMPARISON_NODE_HPP
 
#include <map>
#include <functional>
#include "behaviortree_cpp_v3/condition_node.h"
 
namespace BT
{
template <class T>
class ComparisonNode final : public BT::ConditionNode
{
    public:
        using BT::ConditionNode::ConditionNode;
        ~ComparisonNode() = default;
 
        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("first", "First operand"),
                     BT::InputPort<T>("second", "Second operand"),
                     BT::InputPort<std::string>("comparison_op", "Comparison operator. Valid operators are <, >, <=, >=, == and !=")
                   };
        }
 
        virtual BT::NodeStatus tick() override
        {
            const auto& first         = getInput<T>("first");
            const auto& second        = getInput<T>("second");
            const auto& comparison_op = getInput<std::string>("comparison_op");
 
            if(!first)         { throw BT::RuntimeError { name() + ": " + first.error() };  }
            if(!second)        { throw BT::RuntimeError { name() + ": " + second.error() }; }
            if(!comparison_op) { throw BT::RuntimeError { name() + ": " + comparison_op.error() }; }
 
            try
            {
                return comparison_functions_.at(comparison_op.value())(first.value(), second.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            catch(const std::out_of_range&) { throw BT::RuntimeError { name() + ": invalid comparison operator " + comparison_op.value() }; }
        }
 
    private:
        using Comparison = std::function<bool(const T&, const T&)>;
        const std::map<std::string, Comparison> comparison_functions_
        {
            { "<",  [] (const T& _first, const T& _second) { return _first < _second;  } },
            { ">",  [] (const T& _first, const T& _second) { return _first > _second;  } },
            { "<=", [] (const T& _first, const T& _second) { return _first <= _second; } },
            { ">=", [] (const T& _first, const T& _second) { return _first >= _second; } },
            { "==", [] (const T& _first, const T& _second) { return _first == _second; } },
            { "!=", [] (const T& _first, const T& _second) { return _first != _second; } },
        };
};
 
}
#endif
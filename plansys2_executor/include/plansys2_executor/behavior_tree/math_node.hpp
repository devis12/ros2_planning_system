#ifndef BASIC_MATH_NODE_HPP
#define BASIC_MATH_NODE_HPP
 
#include <map>
#include <functional>
#include "behaviortree_cpp_v3/action_node.h"
 
namespace BT
{
template <class T>
class BasicMathNode final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~BasicMathNode() = default;
 
        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<T>("first", "First operand"),
                     BT::InputPort<T>("second", "Second operand"),
                     BT::InputPort<std::string>("operator", "Math operation. Valid operatores are: +, -, / and *"),
                     BT::OutputPort<T>("output", "Operation result")
                   };
        }
 
        virtual BT::NodeStatus tick() override
        {
            const auto& first   = getInput<T>("first");
            const auto& second  = getInput<T>("second");
            const auto& math_op = getInput<std::string>("operator");
 
            if(!first)   { throw BT::RuntimeError { name() + ": " + first.error()   }; }
            if(!second)  { throw BT::RuntimeError { name() + ": " + second.error()  }; }
            if(!math_op) { throw BT::RuntimeError { name() + ": " + math_op.error() }; }
 
            try
            {
                const auto& result = math_functions_.at(math_op.value())(first.value(), second.value());
                setOutput("output", result);
 
                return BT::NodeStatus::SUCCESS;
            }
            catch(const std::out_of_range&) { throw BT::RuntimeError { name() + ": invalid math operator " + math_op.value() }; }
        }
 
    private:
        using MathFunction = std::function<T(const T&, const T&)>;
        const std::map<std::string, MathFunction> math_functions_
        {
            { "+", [this] (const T& _first, const T& _second) { return _first + _second; }},
            { "-", [this] (const T& _first, const T& _second) { return _first - _second; }},
            { "/", [this] (const T& _first, const T& _second) { return _first / _second; }},
            { "*", [this] (const T& _first, const T& _second) { return _first * _second; }},
        };
};
 
}
#endif
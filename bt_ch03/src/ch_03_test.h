#include <behaviortree_cpp_v3/action_node.h>
using namespace BT;

/* This tutorial will teach you how to deal with ports when its
 *  type is not std::string.
 *  教你使用當資料型態不是std::string時的端口要如何宣告
*/



// We want to be able to use this custom type
// 自定義資料型態
struct Position2D { double x,y; };

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
// 推薦使用(在某個情況下是強制使用)樣板(template)，
// 將字串(std::string)用convertFromString將資料轉變成Position2D內宣告的資料型態
namespace BT
{
template <> inline Position2D convertFromString(StringView str) // StringView是C++11版本的std::string_view
{
    printf("Converting string: \"%s\"\n", str.data() ); // 終端機顯示現在Position2D的內容，目前的型態是std::string

    // real numbers separated by semicolons
    // splitString()為字串分割，根據;來分割字串
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        // 將字串分割完畢之後，就可以使用convertFromeString<資料型態>來將字串改成我們所需要的資料型態
        Position2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        return output;
    }
}
} // end namespace BT

// write int a port
class CalculateGoal: public SyncActionNode
{
public:
    CalculateGoal(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    NodeStatus tick() override
    {
        Position2D mygoal = {1.1, 2.3};// 將Position2D賦予1.1與2.3兩個值
        setOutput("goal", mygoal);
        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts()
    {
        return { OutputPort<Position2D>("goal") };
    }
};


// read from a port
class PrintTarget: public SyncActionNode
{
public:
    PrintTarget(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    NodeStatus tick() override
    {
        auto res = getInput<Position2D>("target");  // 確認運作正常
        if( !res )
        {
            throw RuntimeError("error reading port [target]:", res.error() );
        }
        Position2D goal = res.value();  // Postition2D放入值
        printf("Target positions: [ %.1f, %.1f ]\n", goal.x, goal.y );
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char*  description = "Simply print the target on console...";
        return { InputPort<Position2D>("target", description) };
    }
};
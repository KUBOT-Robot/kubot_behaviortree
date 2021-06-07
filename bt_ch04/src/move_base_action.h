#include <behaviortree_cpp_v3/action_node.h>
using namespace BT;

// Custom type
struct Pose2D
{
    double x, y, theta;
};

namespace BT
{
    template <> inline Pose2D convertFromString(StringView str) // StringView是C++11版本的std::string_view
    {
        auto parts = splitString(str,';');
        if(parts.size() != 3)
        {
            throw RuntimeError("invalid input");
        }
        else{
            Pose2D output;
            output.x =  convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }
}

class MoveBaseAction : public AsyncActionNode
{
  public:
    MoveBaseAction(const std::string& name, const NodeConfiguration& config):
        AsyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
        return{ InputPort<Pose2D>("goal") };
    }

    NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    // 這個成員主要是用來停止節點的執行
    void halt() override
    {
        _halt_requested.store(true);
    }

  private:
    std::atomic_bool _halt_requested;
};

//-------------------------

NodeStatus MoveBaseAction::tick()
{
    Pose2D goal;
    if ( !getInput<Pose2D>("goal", goal))
    {
        //throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n", 
           goal.x, goal.y, goal.theta);

    _halt_requested.store(false);
    int count = 0;

    // Pretend that "computing" takes 250 milliseconds. 假設運行時間需要250ms
    // It is up to you to check periodicall _halt_requested and interrupt
    // this tick() if it is true.
    // 由我們檢查_halt_requested，在它為true時中斷此tick()
    while (!_halt_requested && count++ < 25)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return _halt_requested ? NodeStatus::FAILURE: NodeStatus::SUCCESS;
}
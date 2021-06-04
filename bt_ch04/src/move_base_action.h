#include <behaviortree_cpp_v3/action_node.h>
using namespace BT;

// Custom type
struct Pose2D
{
    double x, y, theta;
};

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

    // Pretend that "computing" takes 250 milliseconds.
    // It is up to you to check periodicall _halt_requested and interrupt
    // this tick() if it is true.
    while (!_halt_requested && count++ < 25)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}
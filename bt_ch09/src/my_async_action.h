#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class MyAsyncAction: public CoroActionNode{
public:
   MyAsyncAction(const std::string& name):CoroActionNode(name, {}){}

private:
   NodeStatus tick() override{
      std::cout << name() << ": Started. Send Request to server." std::endl;

      auto Now = [](){ return std::chrono::high_resolution_clock::now(); };

      TimePoint initial_time = Now();
      TimePoint time_before_reply = initial_time + std::chrono::milliseconds(100);

      int count = 0;
      bool reply_received = false;

      while (!reply_received)
      {
         if(count++ == 0){
            // call this only once
            // 只會執行一次
            std::cout << name() << ": Waitting Reply..." << std::endl;
         }

         // pretend that we received a reply
         // 假設我們收到回覆
         if( Now() >= time_before_reply){
            reply_received = true;
         }

         if(!reply_received){
            // set status to RUNNING and "pause/sleep"
            // If halt() is called, we will not resume execution (stack destroyed)
            // 將狀態設置為RUNNING 和 「暫停中/睡眠中」
            // 如果halt()被調用，將不會恢復執行(堆疊破壞)
            setStatusRunningAndYield();
         }


         // This part of the code is never reached if halt() is invoked,
         // only if reply_received == true;
         // 如果調用了halt()，則這部分的code不會執行
         // 只在reply_received為true的情況下

         std::cout << name() <<": Done. 'Waiting Reply' loop repeated "
                  << count << " times" << std::endl;
         cleanup(false);
         return NodeStatus::SUCCESS;
      }

      // you might want to cleanup differently if it was halted or successful
      // 如果他被停止(halted)或執行成功，你想要以不同的方式清理
      void cleanup(bool halted){
         if(halted){
            std::cout << name() << ": cleaning up after an halt()\n" << std::endl;
         }
         else{
            std::cout << name() << ": cleaning up after SUCCESS \n" << std::endl;
         }
      }

      // 用於暫停的成員
      void halt() override{
         std::cout << name() << ": Halted." << std::endl;
         cleanup(true);
         // Do not forget to call this at the end.
         // 當然請不要忘記調用helt
         CoroActionNode::halt();
      }
   }
};
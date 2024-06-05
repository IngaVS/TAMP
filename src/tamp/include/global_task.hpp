#include <set>
#include "robot_task.hpp"
using namespace std;
class GlobalTask{
public:
	GlobalTask();
	~GlobalTask();
	void GlobalTaskOptimal();
private:
	void ZLQMaterialStatus();

private:
	Config config_;
	vector<RobotTask> robot_tasks_;
};
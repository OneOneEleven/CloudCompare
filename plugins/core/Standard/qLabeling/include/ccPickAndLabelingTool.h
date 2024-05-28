/*
*
*
*
*
* 
*
*/

#include "ccAbstractTool.h"

class ccPickAndLabelingTool :public ccAbstractTool
{
public:
	ccPickAndLabelingTool();
	virtual ~ccPickAndLabelingTool();
 
	//called when the tool is set to active (for initialization)
	void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	void toolDisactivated() override;

	//called when a point in a point cloud gets picked while this tool is active
	void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) override;

	//mouse circle element used for the selection
	//ccMouseCircle* m_mouseCircle = nullptr;
};

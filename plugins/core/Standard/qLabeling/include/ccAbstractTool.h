/**
*
*
*
*
*
*
**/
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>

class ccAbstractTool
{
public:

	virtual ~ccAbstractTool()
	{
	}

	void initializeTool(ccMainAppInterface* app)
	{
		_mainApp = app; //store copy of app
		_glWindow = _mainApp->getActiveGLWindow();
	}

	virtual void toolActivated() { }

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() { }

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccHObject* pickedObject, const CCVector3& P) { }

	//called when a point in a point cloud gets picked while this tool is active
	virtual void pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P) { }

	//called when the selection is changed while this tool is active
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) { }

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	virtual void accept() { }

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	virtual void cancel() { }
 
	virtual bool canUndo() { return false; }
 
	virtual void undo() { }

protected:

	ccAbstractTool() :
		_mainApp(nullptr)
		, _glWindow(nullptr)
	{
	}
	ccMainAppInterface* _mainApp; ///< 主程序接口
  
	ccGLWindowInterface* _glWindow; ///< OpenGL窗口
};

#include "ccPickAndLabelingTool.h"

ccPickAndLabelingTool::ccPickAndLabelingTool()
	: ccAbstractTool()
{
}

ccPickAndLabelingTool::~ccPickAndLabelingTool()
{
}

void ccPickAndLabelingTool::toolActivated()
{
}

void ccPickAndLabelingTool::toolDisactivated()
{
}

void ccPickAndLabelingTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	////get or generate octree
	//ccOctree::Shared oct = cloud->getOctree();
	//if (!oct)
	//{
	//	oct = cloud->computeOctree(); //if the user clicked "no" when asked to compute the octree then tough....
	//	if (!oct)
	//	{
	//		ccLog::Warning("[Point Cloud Labeling Plugin] Failed to compute the cloud octree");
	//		return;
	//	}
	//}

	////nearest neighbour search
	//PointCoordinateType r = static_cast<PointCoordinateType>(m_mouseCircle->getRadiusWorld());
	//unsigned char level = oct->findBestLevelForAGivenNeighbourhoodSizeExtraction(r);
	//CCCoreLib::DgmOctree::NeighboursSet set;
	//int n = oct->getPointsInSphericalNeighbourhood(P, r, set, level);
	////Put data in a point cloud class and encapsulate as a "neighbourhood"
	//CCCoreLib::DgmOctreeReferenceCloud nCloud(&set, n);

	////Fit plane!
	//double rms = 0.0; //output for rms
	//ccFitPlane* pPlane = ccFitPlane::Fit(&nCloud, &rms);

	//if (pPlane) //valid fit
	//{
	//	//we can orient the plane normal to face the viewer
	//	if (m_app->getActiveGLWindow())
	//	{
	//		CCVector3d viewDir = m_app->getActiveGLWindow()->getViewportParameters().getViewDir();
	//		if (pPlane->getNormal().toDouble().dot(viewDir) > 0)
	//		{
	//			pPlane->flip();
	//		}
	//	}

	//	pPlane->updateAttributes(rms, r);

	//	//make plane to add to display
	//	pPlane->setVisible(true);
	//	pPlane->setSelectionBehavior(ccHObject::SELECTION_IGNORED);

	//	//add plane to scene graph
	//	insertPoint->addChild(pPlane);
	//	pPlane->setDisplay(m_app->getActiveGLWindow());
	//	pPlane->prepareDisplayForRefresh_recursive(); //not sure what this does, but it looks like fun

	//	//add plane to TOC
	//	m_app->addToDB(pPlane, false, false, false, false);

	//	//report orientation to console for convenience
	//	m_app->dispToConsole(QString("[ccCompass] Surface orientation estimate = " + pPlane->getName()), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	//}


}

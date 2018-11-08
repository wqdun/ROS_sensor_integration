//
// Created by root on 18-5-20.
//

#include <opencv2/core/persistence.hpp>
#include <GL/glew.h>
#include "MapDrawer.h"
MapDrawer::MapDrawer(const std::string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mScaleic = fSettings["Viewer.Scale"];

}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::SetMarkerPoints(VINS* pEstimator)
{
    std::unique_lock<std::mutex> lock(mMutexMarkers);
    //for(int i = 0; i < pEstimator->mvcurrentmarkers.size(); i++)
    //{
    //    Vector3d ori_marker3d(pEstimator->mvcurrentmarkers[i].at<double>(0, 0), pEstimator->mvcurrentmarkers[i].at<double>(1, 0), pEstimator->mvcurrentmarkers[i].at<double>(2, 0));
    //    Vector3d trans_marker3d = pEstimator->r_drift * ori_marker3d + pEstimator->t_drift;
    //    eimarkers_3d.push_back(trans_marker3d);
    //}
    for(int i = 0; i < pEstimator->mvcurrentmarkers.size(); i++)
    {
          bool findflag = false;
	  for(int j = 0; j < eimarkers_3d.size(); j++)
	  {
               if(eimarkers_3d[j].ID == pEstimator->mvcurrentmarkers[i].ID)
	       {
			
			if(eimarkers_3d[j].maxobsnum < pEstimator->mvcurrentmarkers[i].maxobsnum)
			{
				eimarkers_3d[j].maxobsnum = pEstimator->mvcurrentmarkers[i].maxobsnum;
				eimarkers_3d[j].pos = pEstimator->mvcurrentmarkers[i].pos;				
			}
			findflag = true;
			break;
	       }
	  }
	  if(findflag == false)
	  {
		MarkerPoints3D newdrawmarker;
		newdrawmarker.ID = pEstimator->mvcurrentmarkers[i].ID;
 		newdrawmarker.maxobsnum = pEstimator->mvcurrentmarkers[i].maxobsnum;
		newdrawmarker.pos = pEstimator->mvcurrentmarkers[i].pos;
		eimarkers_3d.push_back(newdrawmarker);
	  }	
    }	
}

void MapDrawer::DisableAllMarkers()
{
	std::unique_lock<std::mutex> lock(mMutexMarkers);
	for(int i = 0; i < eimarkers_3d2.size(); i++)
	{
		eimarkers_3d2[i].maxobsnum = 0;
	}
}

void MapDrawer::SetMarkerPoints2(vector<MarkerPoints3D> vcurrentmarkers)
{
	std::unique_lock<std::mutex> lock(mMutexMarkers);
	//eimarkers_3d2 = vcurrentmarkers;
	for(int i = 0; i < vcurrentmarkers.size(); i++)
    	{
          bool findflag = false;
	  for(int j = 0; j < eimarkers_3d2.size(); j++)
	  {
               if(eimarkers_3d2[j].ID == vcurrentmarkers[i].ID)
	       {
			
			if(eimarkers_3d2[j].maxobsnum < vcurrentmarkers[i].maxobsnum)
			{
				eimarkers_3d2[j].maxobsnum = vcurrentmarkers[i].maxobsnum;
				eimarkers_3d2[j].pos = vcurrentmarkers[i].pos;				
			}
			findflag = true;
			break;
	       }
	  }
	  if(findflag == false)
	  {
		MarkerPoints3D newdrawmarker;
		newdrawmarker.ID = vcurrentmarkers[i].ID;
 		newdrawmarker.maxobsnum = vcurrentmarkers[i].maxobsnum;
		newdrawmarker.pos = vcurrentmarkers[i].pos;
		eimarkers_3d2.push_back(newdrawmarker);
	  }	
    	}	
}

void MapDrawer::DrawMarkerPoints2()
{
    std::unique_lock<std::mutex> lock(mMutexMarkers);
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    for(int i = 0; i < eimarkers_3d2.size() ; i++)
    {
        Vector3d pos = eimarkers_3d2[i].pos;
        glVertex3f(pos[0]/mScaleic,pos[1]/mScaleic,pos[2]/mScaleic);
    }

    glEnd();
}

void MapDrawer::DrawMarkerPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMarkers);
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    for(int i = 0; i < eimarkers_3d.size() ; i++)
    {
        Vector3d pos = eimarkers_3d[i].pos;
        glVertex3f(pos[0]/mScaleic,pos[1]/mScaleic,pos[2]/mScaleic);
    }

    glEnd();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            std::unique_lock<std::mutex> lock(mMutexCamera);
            //cv::Mat InitialTwc = mpMap->GetInitialTransition();
            //cv::Mat InitialTcw = InitialTwc.inv();
            //cv::Mat mtransformedTcw = mCameraPose * InitialTwc;
            cv::Mat mtransformedTcw = cvTwcs[cvTwcs.size()-1].inv();
            Rwc = mtransformedTcw.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mtransformedTcw.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0)/mScaleic;
        M.m[13] = twc.at<float>(1)/mScaleic;
        M.m[14] = twc.at<float>(2)/mScaleic;
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::SetAllFrames(std::vector<DRAWFRAME_DATA> frames_to_draw)
{
    unique_lock<mutex> lock(mMutexCamera);
    cvTwcs.clear();
    for(int i = 0; i < frames_to_draw.size(); i++)
    {
        cv::Mat Rwc;
        cv::Mat twc;
        Rwc = Utility::toCvMat(frames_to_draw[i].R_draw);
        twc = Utility::toCvMat(frames_to_draw[i].P_draw);
        //eigen2cv(frames_to_draw[i].P_draw,twc);
        //eigen2cv(frames_to_draw[i].R_draw,Rwc);
        cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        twc.copyTo(Twc.rowRange(0,3).col(3));
        cvTwcs.push_back(Twc.clone());
    }
}
void MapDrawer::SetAllPoints(std::vector<Vector3d> points_3d)
{
    std::unique_lock<std::mutex> lock(mMutexPoints);
    cvpoints_3d.clear();
    for(int i = 0; i < points_3d.size(); i++)
    {
        cv::Mat cvpt;
        cvpt = Utility::toCvMatd(points_3d[i]);
        //eigen2cv(points_3d[i],cvpt);
        cvpoints_3d.push_back(cvpt.clone());
    }
}

void MapDrawer::DrawMapPoints()
{

    std::unique_lock<std::mutex> lock(mMutexPoints);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(int i = 0; i < cvpoints_3d.size() ; i++)
    {
        cv::Mat pos = cvpoints_3d[i].clone();
        glVertex3f(pos.at<double>(0)/mScaleic,pos.at<double>(1)/mScaleic,pos.at<double>(2)/mScaleic);
    }

    glEnd();
}

void MapDrawer::DrawGrids()
{
    //glLineWidth(mKeyFrameLineWidth);
    //glColor3f(0.5f,0.5f,0.5f);
    //glBegin(GL_LINES);
    const float axis_length = 30;
    const int grid_width = 20;

    ///////////Draw Axis////////////////////////////////////////////////
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(axis_length,0,0);
    glEnd();
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,axis_length,0);
    glEnd();
    glLineWidth(mKeyFrameLineWidth*2);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,0,axis_length);
    glEnd();
    ///////////Draw Grids///////////////////////////////////////////////
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(0.5f,0.5f,0.5f);
    glBegin(GL_LINES);

    for(int vidx = -grid_width; vidx <= grid_width; vidx++)
    {
	if(vidx % 2 == 0)
	{
		glVertex3f((float) vidx , (float) -grid_width, 0);
        	glVertex3f((float) vidx , (float) grid_width, 0);
	}
    }
    //glVertex3f(-grid_width,-grid_width,0);
    //glVertex3f(-grid_width,grid_width,0);
    //glVertex3f(-2.0/3.0*grid_width,-grid_width,0);
    //glVertex3f(-2.0/3.0*grid_width,grid_width,0);
    //glVertex3f(-1.0/3.0*grid_width,-grid_width,0);
    //glVertex3f(-1.0/3.0*grid_width,grid_width,0);
    //glVertex3f(0,-grid_width,0);
    //glVertex3f(0,grid_width,0);
    //glVertex3f(1.0/3.0*grid_width,-grid_width,0);
    //glVertex3f(1.0/3.0*grid_width,grid_width,0);
    //glVertex3f(2.0/3.0*grid_width,-grid_width,0);
    //glVertex3f(2.0/3.0*grid_width,grid_width,0);
    //glVertex3f(grid_width,-grid_width,0);
    //glVertex3f(grid_width,grid_width,0);

    for(int vidx = -grid_width; vidx <= grid_width; vidx++)
    {
	if(vidx % 2 == 0)
	{
		glVertex3f((float) -grid_width , (float) vidx, 0);
        	glVertex3f((float) grid_width , (float) vidx, 0);
	}
    }
    //glVertex3f(-grid_width,-grid_width,0);
    //glVertex3f(grid_width,-grid_width,0);
    //glVertex3f(-grid_width,-2.0/3.0*grid_width,0);
    //glVertex3f(grid_width,-2.0/3.0*grid_width,0);
    //glVertex3f(-grid_width,-1.0/3.0*grid_width,0);
    //glVertex3f(grid_width,-1.0/3.0*grid_width,0);
    //glVertex3f(-grid_width,0,0);
    //glVertex3f(grid_width,0,0);
    //glVertex3f(-grid_width,1.0/3.0*grid_width,0);
    //glVertex3f(grid_width,1.0/3.0*grid_width,0);
    //glVertex3f(-grid_width,2.0/3.0*grid_width,0);
    //glVertex3f(grid_width,2.0/3.0*grid_width,0);
    //glVertex3f(-grid_width,grid_width,0);
    //glVertex3f(grid_width,grid_width,0);
    //////////////////Draw Texts//////////////////////
    const float textscale = grid_width / 3.0;
    glVertex3f(4*textscale,0.2*textscale,0*textscale);
    glVertex3f(5*textscale,0.8*textscale,0*textscale);
    glVertex3f(4*textscale,0.9*textscale,0*textscale);
    glVertex3f(5*textscale,0.1*textscale,0*textscale);
    glVertex3f(-0.9*textscale,5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(-0.1*textscale,5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4*textscale,0*textscale);
    glVertex3f(-0.5*textscale,4.5*textscale,0*textscale);
    glVertex3f(0*textscale,0.3*textscale,5*textscale);
    glVertex3f(0*textscale,0.9*textscale,5*textscale);
    glVertex3f(0*textscale,0.9*textscale,5*textscale);
    glVertex3f(0*textscale,0.2*textscale,4*textscale);
    glVertex3f(0*textscale,0.2*textscale,4*textscale);
    glVertex3f(0*textscale,0.8*textscale,4*textscale);

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    unique_lock<mutex> lock(mMutexCamera);
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;


    if(bDrawKF)
    {
        // camera pose, GPS good, current keyframes
        for(size_t i=cvTwcs.size() - 1; i<cvTwcs.size(); i++)
        {
            if(cvTwcs[i].empty() || cvTwcs[i].dims < 2)
                continue;

            cv::Mat Twc= cvTwcs[i].t();
            cv::Mat t = Twc.row(3).colRange(0,3);
            t = t/mScaleic;
            t.copyTo(Twc.row(3).colRange(0,3));

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,1.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }



    }



    if(cvTwcs.size() > 1 && bDrawGraph)
    {
        for(size_t i = 0; i < cvTwcs.size() - 1; i++)
        {
            if(cvTwcs[i].empty() || cvTwcs[i + 1].empty())
            {
                continue;
            }

            cv::Mat Twc1 = cvTwcs[i].clone();
            cv::Mat Ow1 = Twc1.rowRange(0,3).col(3);
            cv::Mat Twc2 = cvTwcs[i + 1].clone();
            cv::Mat Ow2 = Twc2.rowRange(0,3).col(3);
            float tmp_z1 = Ow1.at<float>(2, 0);
            float tmp_z2 = Ow2.at<float>(2, 0);
            glLineWidth(2 * mGraphLineWidth);
            if(abs(0.5 * (tmp_z1 + tmp_z2)) < 1.5)
                glColor4f(1.0f,0.0f,1.0f,0.6f);
            else if (abs(0.5 * (tmp_z1 + tmp_z2)) < 4.5)
                glColor4f(0.0f,0.0f,1.0f,0.6f);
            else if (abs(0.5 * (tmp_z1 + tmp_z2)) < 7.5)
                glColor4f(1.0f,0.0f,0.0f,0.6f);
            else
                glColor4f(0.0f,0.0f,0.0f,0.6f);
            glBegin(GL_LINES);
            Ow1 = Ow1/mScaleic;
            Ow2 = Ow2/mScaleic;
            glVertex3f(Ow1.at<float>(0),Ow1.at<float>(1),Ow1.at<float>(2));
            glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
            glEnd();
        }
    }
}
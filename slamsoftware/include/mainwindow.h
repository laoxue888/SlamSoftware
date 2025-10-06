#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <QThread>
#include <opencv2/core/core.hpp>
#include "../thirdparty/ORB_SLAM2/include/System.h"

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class SlamWorker : public QThread
{
    Q_OBJECT
public:
    SlamWorker();
    ~SlamWorker();
protected:
    void run() override {
        // 耗时操作
        char **argv = new char *[4];
        argv[0] = (char *)"ORB_SLAM2";
        argv[1] = (char *)"/root/workspace/slamsoftware/thirdparty/Vocabulary/ORBvoc.txt";
        argv[2] = (char *)"/root/workspace/slamsoftware/thirdparty/ORB_SLAM2/configs/Monocular/TUM1.yaml";
        argv[3] = (char *)"/root/workspace/downloads/rgbd_dataset_freiburg1_xyz";

        vector<string> vstrImageFilenames;
        vector<double> vTimestamps;
        string strFile = string(argv[3]) + "/rgb.txt";
        LoadImages(strFile, vstrImageFilenames, vTimestamps);

        int nImages = vstrImageFilenames.size();

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);

        cout << endl
             << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl
             << endl;

        // Main loop
        cv::Mat im;
        for (int ni = 0; ni < nImages; ni++)
        {
            // Read image from file
            im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: "
                     << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
                return;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im, tframe);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages - 1)
                T = vTimestamps[ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestamps[ni - 1];

            if (ttrack < T)
                usleep((T - ttrack) * 1e6);
        }

        // Stop all threads
        SLAM.Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(), vTimesTrack.end());
        float totaltime = 0;
        for (int ni = 0; ni < nImages; ni++)
        {
            totaltime += vTimesTrack[ni];
        }
        cout << "-------" << endl
             << endl;
        cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
        cout << "mean tracking time: " << totaltime / nImages << endl;

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        qDebug() << "Work finished!";
    }
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    QSignalMapper* signalMapper;
    QGraphicsScene* graphicsScene;
    SlamWorker *slamThread;

private slots:
    void switchPage(int pageIndex);
    void actionTestFunc();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H

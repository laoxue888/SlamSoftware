#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 更换page
    // 创建信号映射器
    signalMapper = new QSignalMapper(this);
    // 连接QAction的triggered信号到信号映射器的map方法
    connect(ui->actionORB_SLAM2, &QAction::triggered, signalMapper, qOverload<>(&QSignalMapper::map));
    connect(ui->actionORB_SLAM3, &QAction::triggered, signalMapper, qOverload<>(&QSignalMapper::map));
    signalMapper->setMapping(ui->actionORB_SLAM2, 0);
    connect(signalMapper, &QSignalMapper::mappedInt, this, &MainWindow::switchPage);
    graphicsScene = new QGraphicsScene();

    connect(ui->actionTest, &QAction::triggered, this, &MainWindow::actionTestFunc);
}

    
void MainWindow::actionTestFunc(){
    qDebug() << "Test";

    argv = new char *[4];
    argv[0] = (char *)"ORB_SLAM2";
    argv[1] = (char *)"/root/workspace/slamsoftware/thirdparty/Vocabulary/ORBvoc.txt";
    argv[2] = (char *)"/root/workspace/slamsoftware/thirdparty/ORB_SLAM2/configs/Monocular/TUM1.yaml";
    argv[3] = (char *)"/root/workspace/downloads/rgbd_dataset_freiburg1_xyz";


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

}

void MainWindow::switchPage(int pageIndex)
{
    ui->stackedWidget_2->setCurrentIndex(pageIndex);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}


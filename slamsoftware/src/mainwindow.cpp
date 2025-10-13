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

    //显示图片
    QGraphicsView view(graphicsScene);
    QPixmap pixmap("/root/workspace/slamsoftware/res/images/1305031102.175304.png");
    QGraphicsPixmapItem *item = graphicsScene->addPixmap(pixmap);
    view.show();
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


#include "application.h"
#include "detector.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QDialog>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <iostream>
int main(int argc,char** argv) {
    QApplication app(argc,argv);
    app.setApplicationName("MediaPipe2ManoQt"); app.setApplicationVersion("0.4.0");
    QCommandLineParser parser; parser.setApplicationDescription("Qt C++ MediaPipe 0.10.9 / MANO port");
    parser.addHelpOption(); parser.addVersionOption();
    parser.addOption({"root","Project root (models/configs/assets).","directory"});
    parser.addOption({"mode","viewer, interaction or parallel_ik; omit to show Qt launcher.","mode"});
    parser.addOption({"config","Recursive JSON configuration override.","file"});
    parser.addOption({"frames","Exit after N frames (0: unlimited).","count","0"});
    parser.addOption({"synthetic","Render synthetic MANO geometry without a camera/detector (test only)."});
    parser.addOption({"hidden","Hide both viewports for a bounded camera test (requires --frames)."});
    parser.addOption({"video","Use a local video instead of the camera for repeatable validation.","file"});
    parser.addOption({"capture","Write final Open3D and camera images (requires --frames).","png"});
    parser.addOption({"check","Validate resources and create the native detector; do not open camera/windows."});
    parser.addOption({"benchmark-seconds","Start timing at the first hand detection and exit after N seconds.","seconds"});
    parser.process(app);
    try {
        QString root=parser.value("root");
        if(root.isEmpty()) {
            QDir probe(QCoreApplication::applicationDirPath());
            for(int n=0;n<6;++n) {
                if(QFileInfo::exists(probe.filePath("configs/viewer.json"))) { root=probe.absolutePath(); break; }
                if(!probe.cdUp()) break;
            }
        }
        if(root.isEmpty()) throw std::runtime_error("Cannot locate project assets; supply --root");
        root=QFileInfo(root).absoluteFilePath();
        QString overridePath=parser.value("config");
        if(!overridePath.isEmpty()) overridePath=QFileInfo(overridePath).absoluteFilePath();
        QString videoPath=parser.value("video");
        if(!videoPath.isEmpty()) {
            videoPath=QFileInfo(videoPath).absoluteFilePath();
            if(parser.isSet("synthetic")) throw std::runtime_error("--video and --synthetic cannot be combined");
        }
        bool ok=false; int frames=parser.value("frames").toInt(&ok);
        if(!ok || frames<0) throw std::runtime_error("--frames requires a nonnegative integer");
        double benchmarkSeconds=0;
        if(parser.isSet("benchmark-seconds")) {
            benchmarkSeconds=parser.value("benchmark-seconds").toDouble(&ok);
            if(!ok || benchmarkSeconds<=0) throw std::runtime_error("--benchmark-seconds requires a positive number");
            if(parser.isSet("synthetic")) throw std::runtime_error("--benchmark-seconds requires a camera or video input");
            if(frames>0) throw std::runtime_error("--benchmark-seconds and --frames cannot be combined");
        }
        if(parser.isSet("synthetic") && frames==0) frames=3;
        if(parser.isSet("hidden") && frames==0) throw std::runtime_error("--hidden requires --frames");
        QString capture=parser.value("capture");
        if(!capture.isEmpty()) {
            if(frames==0) throw std::runtime_error("--capture requires --frames");
            capture=QFileInfo(capture).absoluteFilePath();
        }
        // MediaPipe 0.10.9 model paths are relative to its resource root.
        if(!QDir::setCurrent(root+"/assets")) throw std::runtime_error("Missing assets directory");
        std::map<QString,QString> overrides;
        overrides["interaction"]=overridePath; overrides["parallel_ik"]=overridePath; overrides["viewer"]=overridePath;
        auto run=[&](const QString& mode) {
            if(mode!="viewer" && mode!="interaction" && mode!="parallel_ik") throw std::runtime_error("--mode must be viewer, interaction or parallel_ik");
            auto config=m2m::loadConfig(root,mode,overrides[mode]);
            if(parser.isSet("check")) {
                m2m::ManoModel left((root+"/models/MANO_LEFT.bin").toStdString());
                m2m::ManoModel right((root+"/models/MANO_RIGHT.bin").toStdString());
                auto d=config["detector"].toObject();
                m2m::Detector detector(root,d["model_complexity"].toInt(),d["max_hands"].toInt(),
                    d["xnnpack_threads"].toInt(),
                    d["min_detection_confidence"].toDouble(),d["min_tracking_confidence"].toDouble());
                cv::Mat blank=cv::Mat::zeros(480,640,CV_8UC3);
                if(!detector.process(blank).empty()) throw std::runtime_error("Unexpected hand detection on black frame");
                std::cout<<"PASS: config, MANO resources, native detector initialization and no-hand frame\n";
                return 0;
            }
            return m2m::runApplication(root,config,mode!="viewer",frames,parser.isSet("synthetic"),capture,parser.isSet("hidden"),videoPath,benchmarkSeconds);
        };
        if(parser.isSet("mode") || parser.isSet("check") || parser.isSet("synthetic"))
            return run(parser.isSet("mode")?parser.value("mode"):"interaction");
        QWidget launcher; launcher.setWindowTitle("MediaPipe → MANO"); launcher.resize(420,210);
        QVBoxLayout layout(&launcher);
        QLabel description(QString::fromUtf8("选择运行模式"));
        description.setWordWrap(true); layout.addWidget(&description);
        QPushButton viewer(QString::fromUtf8("手部交互预览")),parallelIk(QString::fromUtf8("并行 IK 测试")),configButton(QString::fromUtf8("选择配置"));
        layout.addWidget(&viewer); layout.addWidget(&parallelIk); layout.addWidget(&configButton);
        auto start=[&](const QString& mode) {
            launcher.hide();
            try { run(mode); } catch(const std::exception& e) { QMessageBox::critical(&launcher,"MediaPipe2ManoQt",QString::fromUtf8(e.what())); }
            launcher.show();
        };
        QObject::connect(&viewer,&QPushButton::clicked,[&]{start("interaction");});
        QObject::connect(&parallelIk,&QPushButton::clicked,[&]{start("parallel_ik");});
        QObject::connect(&configButton,&QPushButton::clicked,[&]{
            QDialog dialog(&launcher); dialog.setWindowTitle(QString::fromUtf8("选择配置"));
            QFormLayout form(&dialog);
            QLineEdit preview(overrides["interaction"]), parallel(overrides["parallel_ik"]);
            preview.setPlaceholderText(root+"/configs/interaction.json");
            parallel.setPlaceholderText(root+"/configs/parallel_ik.json");
            auto row=[&](const QString& title,QLineEdit& field) {
                auto container=new QWidget(&dialog); auto layout=new QHBoxLayout(container);
                layout->setContentsMargins(0,0,0,0); layout->addWidget(&field);
                auto browse=new QPushButton(QString::fromUtf8("浏览"),container); layout->addWidget(browse);
                QObject::connect(browse,&QPushButton::clicked,&dialog,[&,edit=&field] {
                    auto file=QFileDialog::getOpenFileName(&dialog,QString::fromUtf8("选择配置"),root+"/configs","JSON (*.json)");
                    if(!file.isEmpty()) edit->setText(file);
                });
                form.addRow(title,container);
            };
            row(QString::fromUtf8("手部交互预览"),preview); row(QString::fromUtf8("并行 IK 测试"),parallel);
            QDialogButtonBox buttons(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
            form.addRow(&buttons);
            QObject::connect(&buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
            QObject::connect(&buttons,&QDialogButtonBox::accepted,&dialog,[&] {
                try {
                    m2m::loadConfig(root,"interaction",preview.text().trimmed());
                    m2m::loadConfig(root,"parallel_ik",parallel.text().trimmed());
                    overrides["interaction"]=preview.text().trimmed(); overrides["parallel_ik"]=parallel.text().trimmed();
                    dialog.accept();
                } catch(const std::exception& e) { QMessageBox::warning(&dialog,QString::fromUtf8("配置错误"),QString::fromUtf8(e.what())); }
            });
            dialog.resize(700,150); dialog.exec();
        });
        launcher.show(); return app.exec();
    } catch(const std::exception& e) { std::cerr<<"ERROR: "<<e.what()<<'\n'; return 1; }
}

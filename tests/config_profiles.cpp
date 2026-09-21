#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <stdexcept>

namespace {
QJsonObject readObject(const QString& path) {
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) throw std::runtime_error(("Cannot open "+path).toStdString());
    QJsonParseError error;
    auto document=QJsonDocument::fromJson(file.readAll(),&error);
    if(error.error!=QJsonParseError::NoError || !document.isObject())
        throw std::runtime_error(("Invalid JSON: "+path).toStdString());
    return document.object();
}

QJsonObject merge(QJsonObject base,const QJsonObject& overrideObject) {
    for(auto it=overrideObject.begin();it!=overrideObject.end();++it) {
        if(it.value().isObject() && base.value(it.key()).isObject())
            base[it.key()]=merge(base.value(it.key()).toObject(),it.value().toObject());
        else base[it.key()]=it.value();
    }
    return base;
}

int integer(const QJsonObject& config,const char* group,const char* key) {
    auto value=config.value(group).toObject().value(key);
    if(!value.isDouble()) throw std::runtime_error(std::string("Missing numeric config: ")+group+"."+key);
    return value.toInt();
}

QByteArray withoutXnnpackThreads(QJsonObject config) {
    auto detector=config.value("detector").toObject();
    detector.remove("xnnpack_threads");
    config["detector"]=detector;
    return QJsonDocument(config).toJson(QJsonDocument::Compact);
}

void checkProfile(const QJsonObject& config,int xnnpackThreads,int jacobianWorkers,const char* name) {
    if(integer(config,"detector","xnnpack_threads")!=xnnpackThreads)
        throw std::runtime_error(std::string(name)+": unexpected xnnpack_threads");
    if(integer(config,"mano","jacobian_workers")!=jacobianWorkers)
        throw std::runtime_error(std::string(name)+": unexpected jacobian_workers");
}
}

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        if(argc!=2) throw std::runtime_error("Usage: config_profiles_tests project-root");
        const QString root=QString::fromLocal8Bit(argv[1]);
        const auto interaction=readObject(root+"/configs/interaction.json");
        const auto parallelIk=readObject(root+"/configs/parallel_ik.json");
        checkProfile(interaction,4,1,"interaction.json");
        checkProfile(parallelIk,4,4,"parallel_ik.json");

        struct Variant { const char* file; int threads; };
        for(const auto& variant:{Variant{"interaction_4threads.json",4},Variant{"interaction_8threads.json",8},Variant{"interaction_16threads.json",16}}) {
            const auto config=merge(interaction,readObject(root+"/configs/"+variant.file));
            checkProfile(config,variant.threads,1,variant.file);
            if(withoutXnnpackThreads(config)!=withoutXnnpackThreads(interaction))
                throw std::runtime_error(std::string(variant.file)+": must differ from interaction.json only by xnnpack_threads");
        }
        for(const auto& variant:{Variant{"parallel_ik_8threads.json",8},Variant{"parallel_ik_16threads.json",16}}) {
            const auto config=merge(parallelIk,readObject(root+"/configs/"+variant.file));
            checkProfile(config,variant.threads,4,variant.file);
            if(withoutXnnpackThreads(config)!=withoutXnnpackThreads(parallelIk))
                throw std::runtime_error(std::string(variant.file)+": must differ from parallel_ik.json only by xnnpack_threads");
        }
        struct Benchmark { const char* file; int xnnpackThreads; int jacobianWorkers; };
        for(const auto& benchmark:{
            Benchmark{"xnnpack1_jacobian4.json",1,4},Benchmark{"xnnpack2_jacobian1.json",2,1},
            Benchmark{"xnnpack2_jacobian2.json",2,2},Benchmark{"xnnpack2_jacobian4.json",2,4},
            Benchmark{"xnnpack4_jacobian0.json",4,0},Benchmark{"xnnpack4_jacobian1.json",4,1},
            Benchmark{"xnnpack4_jacobian2.json",4,2},Benchmark{"xnnpack4_jacobian4.json",4,4},
            Benchmark{"xnnpack4_jacobian8.json",4,8},Benchmark{"xnnpack8_jacobian1.json",8,1},
            Benchmark{"xnnpack8_jacobian2.json",8,2},Benchmark{"xnnpack8_jacobian4.json",8,4},
            Benchmark{"xnnpack16_jacobian4.json",16,4}}) {
            const auto config=merge(parallelIk,readObject(root+"/configs/benchmarks/"+benchmark.file));
            checkProfile(config,benchmark.xnnpackThreads,benchmark.jacobianWorkers,benchmark.file);
        }
        std::cout<<"PASS: interaction and parallel_ik configuration profiles\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';
        return 1;
    }
}

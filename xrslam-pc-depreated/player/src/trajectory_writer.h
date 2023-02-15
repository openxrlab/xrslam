#ifndef XRSLAM_PC_TRAJECTORY_WRITER_H
#define XRSLAM_PC_TRAJECTORY_WRITER_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <xrslam/xrslam.h>

class TrajectoryWriter {
  public:
    virtual ~TrajectoryWriter() = default;
    virtual void write_pose(const double &t, const xrslam::Pose &pose) = 0;
};

class ConsoleTrajectoryWriter : public TrajectoryWriter {
  public:
    ~ConsoleTrajectoryWriter() { fprintf(stdout, "\n"); }

    void write_pose(const double &t, const xrslam::Pose &pose) override {
        fprintf(stdout,
                "\r% 18.3f: p = (% 10.3f,% 10.3f,% 10.3f), q = (% 7.5f,% "
                "7.5f,% 7.5f,% 7.5f)",
                t, pose.p.x(), pose.p.y(), pose.p.z(), pose.q.x(), pose.q.y(),
                pose.q.z(), pose.q.w());
    }
};

class CsvTrajectoryWriter : public TrajectoryWriter {
    FILE *file;

  public:
    CsvTrajectoryWriter(const std::string &filename) {
        if (!(file = fopen(filename.c_str(), "w"))) {
            throw "Cannot open file";
        }
    }

    ~CsvTrajectoryWriter() {
        fflush(file);
        fclose(file);
    }

    void write_pose(const double &t, const xrslam::Pose &pose) override {
        fprintf(file, "%.18e,%.9e,%.9e,%.9e,%.7e,%.7e,%.7e,%.7e\n", t,
                pose.p.x(), pose.p.y(), pose.p.z(), pose.q.x(), pose.q.y(),
                pose.q.z(), pose.q.w());
        fflush(file);
    }
};

class TumTrajectoryWriter : public TrajectoryWriter {
    FILE *file;

  public:
    TumTrajectoryWriter(const std::string &filename) {
        if (!(file = fopen(filename.c_str(), "w"))) {
            throw "Cannot open file";
        }
    }

    ~TumTrajectoryWriter() {
        fflush(file);
        fclose(file);
    }

    void write_pose(const double &t, const xrslam::Pose &pose) override {
        fprintf(file, "%.18e %.9e %.9e %.9e %.7e %.7e %.7e %.7e\n", t,
                pose.p.x(), pose.p.y(), pose.p.z(), pose.q.x(), pose.q.y(),
                pose.q.z(), pose.q.w());
        fflush(file);
    }
};

#endif // XRSLAM_PC_TRAJECTORY_WRITER_H

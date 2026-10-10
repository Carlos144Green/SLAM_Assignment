from __future__ import print_function #handle print in 2.x python
from . import IccPlots as plots
import sm
import numpy as np
import pylab as pl
import sys
import subprocess
import yaml
import time
from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import io
try:
    # Python 2
    from cStringIO import StringIO
except ImportError:
    # Python 3
    from io import StringIO
import matplotlib
import matplotlib.patches as patches

# make numpy print prettier
np.set_printoptions(suppress=True)


def _ensure_pdf_backend():
    backend = matplotlib.get_backend().lower()
    if "agg" in backend:
        return
    try:
        matplotlib.use("Agg", force=True)
    except Exception:
        pass


def _new_report_figure(figure_number, readable=False):
    if readable:
        return pl.figure(figure_number, figsize=(11, 8.5))
    return pl.figure(figure_number)


def _build_report_figures(cself, offset=3010, readable=False):
    figs = list()

    sstream = StringIO()
    printResultTxt(cself, sstream)
    text = [line for line in StringIO(sstream.getvalue())]
    lines_per_page = 30 if readable else 35
    text_font_size = 9 if readable else 7

    while True:
        fig = _new_report_figure(offset, readable=readable)
        offset += 1

        left, width = 0.05, 1.0
        bottom, height = -0.05, 1.0
        ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
        patches.Rectangle(
            (left, bottom),
            width,
            height,
            fill=False,
            transform=ax.transAxes,
            clip_on=False,
            edgecolor="none",
        )
        pl.axis("off")

        def print_text(text_block):
            ax.text(
                left,
                bottom + height,
                text_block,
                fontsize=text_font_size,
                horizontalalignment="left",
                verticalalignment="top",
                transform=ax.transAxes,
                wrap=True,
            )

        if len(text) > lines_per_page:
            print_text("".join(text[0:lines_per_page]))
            figs.append(fig)
            text = text[lines_per_page:]
        else:
            print_text("".join(text[0:]))
            figs.append(fig)
            break

    trajectory_figure = _new_report_figure(1003, readable=readable)
    plotTrajectory(
        cself,
        fno=trajectory_figure.number,
        clearFigure=False,
        title="imu0: estimated poses",
    )
    figs.append(trajectory_figure)

    for iidx, imu in enumerate(cself.ImuList):
        imu_plotters = (
            (plots.plotIMURates, False),
            (plots.plotAccelerations, True),
            (plots.plotAccelErrorPerAxis, True),
            (plots.plotAccelBias, True),
            (plots.plotAngularVelocities, True),
            (plots.plotGyroErrorPerAxis, True),
            (plots.plotAngularVelocityBias, True),
        )
        for plotter, label_axes in imu_plotters:
            figure = _new_report_figure(offset + iidx, readable=readable)
            plotter(cself, iidx, fno=figure.number, noShow=True)
            if readable and label_axes:
                plots.label_imu_subplot_axes(figure)
            figs.append(figure)
            offset += len(cself.ImuList)

    if cself.CameraChain:
        for cidx, cam in enumerate(cself.CameraChain.camList):
            figure = _new_report_figure(offset + cidx, readable=readable)
            title = "cam{0}: reprojection errors".format(cidx)
            plots.plotReprojectionScatter(
                cself, cidx, fno=figure.number, noShow=True, title=title)
            figs.append(figure)
            offset += len(cself.CameraChain.camList)

    return figs


def _write_report_pdf(figs, filename, readable=False):
    pdf = PdfPages(filename)
    for figure in figs:
        if readable:
            pdf.savefig(figure, bbox_inches="tight")
        else:
            pdf.savefig(figure)
    pdf.close()
    for figure in figs:
        pl.close(figure)


def plotTrajectory(cself, fno=1, clearFigure=True, title=""):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)

    size = 0.05
    a3d = f.add_subplot(111, projection='3d')

    # get times we will evaulate at (fixed frequency)
    imu = cself.ImuList[0]
    bodyspline = cself.poseDv.spline()
    times_imu = np.array([im.stamp.toSec() + imu.timeOffset for im in imu.imuData \
                      if im.stamp.toSec() + imu.timeOffset > bodyspline.t_min() \
                      and im.stamp.toSec() + imu.timeOffset < bodyspline.t_max() ])
    times = np.arange(np.min(times_imu), np.max(times_imu), 1.0/10.0)
    
    #plot each pose
    traj_max = np.array([-9999.0, -9999.0, -9999.0])
    traj_min = np.array([9999.0, 9999.0, 9999.0])
    T_last = None
    for time in times:
        position =  bodyspline.position(time)
        orientation = sm.r2quat(bodyspline.orientation(time))
        T = sm.Transformation(orientation, position)
        sm.plotCoordinateFrame(a3d, T.T(), size=size)
        # record min max
        traj_max = np.maximum(traj_max, position)
        traj_min = np.minimum(traj_min, position)
        # compute relative change between
        if T_last != None:
            pos1 = T_last.t()
            pos2 = T.t()
            a3d.plot3D([pos1[0], pos2[0]],[pos1[1], pos2[1]],[pos1[2], pos2[2]],'k-', linewidth=1)
        T_last = T

    #TODO: should also plot the target board here (might need to transform into imu0 grav?)

    a3d.auto_scale_xyz([traj_min[0]-size, traj_max[0]+size], [traj_min[1]-size, traj_max[1]+size], [traj_min[2]-size, traj_max[2]+size])


def printErrorStatistics(cself, dest=sys.stdout):
    # Reprojection errors
    print("Normalized Residuals\n----------------------------", file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.sqrt(rerr.evaluateError()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print("Reprojection error (cam{0}):     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) ), file=dest)
        else:
            print("Reprojection error (cam{0}):     no corners".format(cidx), file=dest)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.gyroErrors ])
        print("Gyroscope error (imu{0}):        mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        # Accelerometer errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.accelErrors ])
        print("Accelerometer error (imu{0}):    mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)

    print("", file=dest)
    print("Residuals\n----------------------------", file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.linalg.norm(rerr.error()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print("Reprojection error (cam{0}) [px]:     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) ), file=dest)
        else:
            print("Reprojection error (cam{0}) [px]:     no corners".format(cidx), file=dest)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.gyroErrors ])
        print("Gyroscope error (imu{0}) [rad/s]:     mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        # Accelerometer errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.accelErrors ])
        print("Accelerometer error (imu{0}) [m/s^2]: mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)

def printGravity(cself):
    print("")
    print("Gravity vector: (in target coordinates): [m/s^2]")
    print(cself.gravityDv.toEuclidean())

def printResults(cself, withCov=False):
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T_cam_b = cself.CameraChain.getResultTrafoImuToCam(camNr)

        print("")
        print("Transformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): ".format(camNr))
        if withCov and camNr==0:
            print("    quaternion: ", T_cam_b.q(), " +- ", cself.std_trafo_ic[0:3])
            print("    translation: ", T_cam_b.t(), " +- ", cself.std_trafo_ic[3:])
        print(T_cam_b.T())
        
        if not cself.noTimeCalibration:
            print("")
            print("cam{0} to imu0 time: [s] (t_imu = t_cam + shift)".format(camNr))
            print(cself.CameraChain.getResultTimeShift(camNr), end=' ')
            
            if withCov:
                print(" +- ", cself.std_times[camNr])
            else:
                print("")

    print("")
    for (imuNr, imu) in enumerate(cself.ImuList):
        print("IMU{0}:\n".format(imuNr), "----------------------------")
        imu.getImuConfig().printDetails()
            
def printBaselines(self):
    #print all baselines in the camera chain
    if nCams > 1:
        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            
            if cself.CameraChain.camList[camNr+1].T_extrinsic_fixed:
                isFixed = "(fixed to external data)"
            else:
                isFixed = ""
            
            print("")
            print("Baseline (cam{0} to cam{1}): [m] {2}".format(camNr, camNr+1, isFixed))
            print(T.T())
            print(baseline, "[m]")
    


def generateReport(cself, filename="report.pdf", showOnScreen=False):
    _ensure_pdf_backend()
    figs = _build_report_figures(cself, offset=3010, readable=False)
    _write_report_pdf(figs, filename, readable=False)
    if showOnScreen:
        print(
            "Interactive wx report viewer is disabled in this Kalibr build. "
            "Open the PDF reports instead: {0}".format(filename))


def generateReadableReport(cself, filename="report-readable.pdf"):
    _ensure_pdf_backend()
    figs = _build_report_figures(cself, offset=4010, readable=True)
    _write_report_pdf(figs, filename, readable=True)

def exportPoses(cself, filename="poses_imu0.csv"):
    
    # Append our header, and select times at IMU rate
    f = open(filename, 'w')
    print("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []", file=f)
    imu = cself.ImuList[0]
    bodyspline = cself.poseDv.spline()
    times = np.array([im.stamp.toSec() + imu.timeOffset for im in imu.imuData \
                      if im.stamp.toSec() + imu.timeOffset > bodyspline.t_min() \
                      and im.stamp.toSec() + imu.timeOffset < bodyspline.t_max() ])

    # Times are in nanoseconds -> convert to seconds
    # Use the ETH groundtruth csv format [t,q,p,v,bg,ba]
    for time in times:
        position =  bodyspline.position(time)
        orientation = sm.r2quat(bodyspline.orientation(time))
        print("{:.0f},".format(1e9 * time) + ",".join(map("{:.6f}".format, position)) \
               + "," + ",".join(map("{:.6f}".format, orientation)) , file=f)

def saveResultTxt(cself, filename='cam_imu_result.txt'):
    f = open(filename, 'w')
    printResultTxt(cself, stream=f)

def printResultTxt(cself, stream=sys.stdout):
    
    print("Calibration results", file=stream)
    print("===================", file=stream)   
    printErrorStatistics(cself, stream)
  
    # Calibration results
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print("", file=stream)
        print("Transformation (cam{0}):".format(camNr), file=stream)
        print("-----------------------", file=stream)
        print("T_ci:  (imu0 to cam{0}): ".format(camNr), file=stream)   
        print(T.T(), file=stream)
        print("", file=stream)
        print("T_ic:  (cam{0} to imu0): ".format(camNr), file=stream)   
        print(T.inverse().T(), file=stream)
    
        # Time
        print("", file=stream)
        print("timeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)".format(camNr), file=stream)
        print(cself.CameraChain.getResultTimeShift(camNr), file=stream)
        print("", file=stream)

    #print all baselines in the camera chain
    if nCams > 1:
        print("Baselines:", file=stream)
        print("----------", file=stream)

        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            print("Baseline (cam{0} to cam{1}): ".format(camNr, camNr+1), file=stream)
            print(T.T(), file=stream)
            print("baseline norm: ", baseline,  "[m]", file=stream)
            print("", file=stream)
    
    # Gravity
    print("", file=stream)
    print("Gravity vector in target coords: [m/s^2]", file=stream)
    print(cself.gravityDv.toEuclidean(), file=stream)
    
    print("", file=stream)
    print("", file=stream)
    print("Calibration configuration", file=stream)
    print("=========================", file=stream)
    print("", file=stream)

    for camNr, cam in enumerate( cself.CameraChain.camList ):
        print("cam{0}".format(camNr), file=stream)
        print("-----", file=stream)
        cam.camConfig.printDetails(stream)
        cam.targetConfig.printDetails(stream)
        print("", file=stream)
    
    print("", file=stream)
    print("", file=stream)
    print("IMU configuration", file=stream)
    print("=================", file=stream)
    print("", file=stream)
    for (imuNr, imu) in enumerate(cself.ImuList):
        print("IMU{0}:\n".format(imuNr), "----------------------------", file=stream)
        imu.getImuConfig().printDetails(stream)
        print("", file=stream)

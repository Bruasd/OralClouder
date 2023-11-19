#Author-Nuofan Qiu
#Date-20230706
#Description-
# running in Fusion 360

import adsk.core, adsk.fusion, adsk.cam, traceback
import os
import time
import math
import json

with open('JsonData.json') as f:
    data = json.load(f)

n = data['total']
r1 = data['r1']
h1 = data['h1']
positions = []
vectors = []
for i in range(1, n + 1):
    position = data['position{}'.format(i)]  # 读取位置向量
    positions.append(position)  # 将向量添加到列表中
    vector = data['vector{}'.format(i)]  # 读取法向向量
    vectors.append(vector)  # 将向量添加到列表中

positions[1] = [x/10 for x in positions[1]] # mm -> cm
positions[2] = [x/10 for x in positions[2]] # mm -> cm
positions[3] = [x/10 for x in positions[3]] # mm -> cm
positions[4] = [x/10 for x in positions[4]] # mm -> cm
positions[5] = [x/10 for x in positions[5]] # mm -> cm
positions[6] = [x/10 for x in positions[6]] # mm -> cm

r1 = r1/10 # mm -> cm
h1 = h1/10 # mm -> cm

def getT(location: list, vector) -> adsk.core.Matrix3D:
    """
    return a Matrix3D object as the homogeneous matrix
    location is the origin point
    vector is the normal vector
    """
    T = adsk.core.Matrix3D.create()

    fromVec = adsk.core.vectors[3].create(0.0, 0.0, 1.0)
    toVec = adsk.core.vectors[3].create(vector[0], vector[1], vector[2])

    T.setToRotateTo(fromVec, toVec)
    T.setCell(0, 3, location[0])
    T.setCell(1, 3, location[1])
    T.setCell(2, 3, location[2])

    return T

def setLocation(location)->adsk.core.Matrix3D:
    """
    return a Matrix3D object represent different location
    """
    M = adsk.core.Matrix3D.create()
    M.setCell(0, 3, location[0])
    M.setCell(1, 3, location[1])
    M.setCell(2, 3, location[2])

    return M

def generateCylinder(r, h, comp: adsk.fusion.Component) -> adsk.fusion.BRepBodies:
    """
    generate a cylinder with radius: r-cm, height: h-cm
    """
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    centerPoint = adsk.core.Point3D.create(0.0, 0.0, 0.0)
    circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, r)
    profile = sketch.profiles.item(0)
    extrudes = comp.features.extrudeFeatures
    # extInput = extrudes.createInput(circle, adsk.fusion.FeatureOperations.NewComponentFeatureOperation)
    distance = adsk.core.ValueInput.createByReal(h)
    extrude = extrudes.addSimple(profile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    return extrude.bodies

def get_revolute_axis(
    comp: adsk.fusion.Component,
    sPoint: adsk.core.Point3D,
    ePoint: adsk.core.Point3D,
) -> adsk.fusion.ConstructionAxis:

    app: adsk.core.Application = adsk.core.Application.get()
    des: adsk.fusion.Design = app.activeProduct

    vec: adsk.core.vectors[3] = sPoint.vectorTo(ePoint)
    vec.normalize()

    infinite: adsk.core.InfiniteLine3D = adsk.core.InfiniteLine3D.create(
        sPoint,
        vec,
    )

    baseFeat: adsk.fusion.BaseFeature = None
    if des.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        baseFeat = comp.features.baseFeatures.add()

    axes: adsk.fusion.ConstructionAxes = comp.constructionAxes
    axisIpt: adsk.fusion.ConstructionAxisInput = axes.createInput()
    axis: adsk.fusion.ConstructionAxis = None
    if baseFeat:
        try:
            baseFeat.startEdit()
            axisIpt.setByLine(infinite)
            axis = axes.add(axisIpt)
        except:
            pass
        finally:
            baseFeat.finishEdit()
    else:
        axisIpt.setByLine(infinite)
        axis = axes.add(axisIpt)

    return axis

def generateBall(r, comp: adsk.fusion.Component) -> adsk.fusion.BRepBodies:
    """
    gerenate a ball with raduis: r-cm
    """
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    centerPoint = adsk.core.Point3D.create(0.0, 0.0, 0.0)
    circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, r)

    revolveFeatures = comp.features.revolveFeatures
    # # Create the axis for the revolve operation
    # z_axis = comp.zConstructionAxis
    # axis_line = sketch.sketchCurves.sketchLines.addByTwoPoints(centerPoint, adsk.core.Point3D.create(0, 0, 1))
    # axis = get_revolute_axis(comp, adsk.core.Point3D.create(0.0, 0.0, 0.0), adsk.core.Point3D.create(0.0, 0.0, 0.1))
    # Draw a line to use as the axis of revolution.
    # https://forums.autodesk.com/t5/fusion-360-api-and-scripts/creating-a-sphere/m-p/6739010
    lines = sketch.sketchCurves.sketchLines
    axisLine = lines.addByTwoPoints(adsk.core.Point3D.create(-3, 0, 0), adsk.core.Point3D.create(3, 0, 0))
    # revolveInput = revolveFeatures.createInput(sketch.profiles.item(0), comp.xConstructionAxis, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    revolveInput = revolveFeatures.createInput(sketch.profiles.item(0), axisLine, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    angle = adsk.core.ValueInput.createByString("360 deg")
    revolveInput.setAngleExtent(False, angle)
    revolveFeature = revolveFeatures.add(revolveInput)

    # sphere_features = comp.features.sphereFeatures
    # sphere_input = sphere_features.createInput()
    # sphere_input.setByCenterRadius(adsk.core.Point3D.create(0, 0, 0), r)  # Replace with your desired center and radius values
    # sphere_feature = sphere_features.add(sphere_input)

    return revolveFeature.bodies
    # return sphere_feature.bodies

def getNorm(vec) -> float:
    """
    get the length(norm) of a vector
    """
    sum = 0
    for i in vec:
        sum += i**2

    return math.sqrt(sum)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        rootComp = design.rootComponent
        importManager = app.importManager
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        textPalette = ui.palettes.itemById("TextCommands")
        if not textPalette.isVisible:
            textPalette.isVisible = True


        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_file = current_dir + "/jointmodel.step"
        # f3dOptions = importManager.createFusionArchiveImportOptions(model_file)
        # f3dOptions.isViewFit = True
        # impObj = importManager.importToTarget2(f3dOptions, rootComp)

        stepOptions = importManager.createSTEPImportOptions(model_file)
        stepOptions.isViewFit = True
        impObj = importManager.importToTarget2(stepOptions, rootComp)

        T1 = getT(positions[1], vectors[1])
        modelOcc1: adsk.fusion.Occurrence = impObj.item(0)
        modelCom1 = modelOcc1.component
        modelOcc1.transform2 = T1
        modelCom1.name = "Teeth1"
        (origin1, xAxis1, yAxis1, zAxis1) = modelOcc1.transform2.getAsCoordinateSystem()
        # textPalette.writeText("origin point: {}, {}, {}".format(origin1.x, origin1.y, origin1.z))
        # cylStartPoint = [origin1.x + h1*zAxis1.x, origin1.y + h1*zAxis1.y, origin1.z + h1*zAxis1.z]
        time.sleep(1.5)

        T2 = getT(positions[2], vectors[2])
        modelOcc2 = rootComp.occurrences.addNewComponentCopy(modelCom1, T2)
        modelCom2 = modelOcc2.component
        modelCom2.name = "Teeth2"
        (origin2, xAxis2, yAxis2, zAxis2) = modelOcc2.transform2.getAsCoordinateSystem()
        time.sleep(1.5)

        T3 = getT(positions[3], vectors[3])
        modelOcc3 = rootComp.occurrences.addNewComponentCopy(modelCom1, T3)
        modelCom3 = modelOcc3.component
        modelCom3.name = "Teeth3"
        (origin3, xAxis3, yAxis3, zAxis3) = modelOcc3.transform2.getAsCoordinateSystem()
        time.sleep(1.5)

        T4 = getT(positions[4], vectors[4])
        modelOcc4 = rootComp.occurrences.addNewComponentCopy(modelCom1, T4)
        modelCom4 = modelOcc4.component
        modelCom4.name = "Teeth4"
        (origin4, xAxis4, yAxis4, zAxis4) = modelOcc4.transform2.getAsCoordinateSystem()
        time.sleep(1.5)

        T5 = getT(positions[5], vectors[5])
        modelOcc5 = rootComp.occurrences.addNewComponentCopy(modelCom1, T5)
        modelCom5 = modelOcc5.component
        modelCom5.name = "Teeth5"
        (origin5, xAxis5, yAxis5, zAxis5) = modelOcc5.transform2.getAsCoordinateSystem()
        time.sleep(1.5)

        T6 = getT(positions[6], vectors[6])
        modelOcc6 = rootComp.occurrences.addNewComponentCopy(modelCom1, T6)
        modelCom6 = modelOcc6.component
        modelCom6.name = "Teeth6"
        (origin6, xAxis6, yAxis6, zAxis6) = modelOcc6.transform2.getAsCoordinateSystem()
        time.sleep(1.5)

        cylStartPoint_1 = [origin1.x + h1 * zAxis1.x, origin1.y + h1 * zAxis1.y, origin1.z + h1 * zAxis1.z]
        cylEndPoint_1 = [origin2.x + h1 * zAxis2.x, origin2.y + h1 * zAxis2.y, origin2.z + h1 * zAxis2.z]
        # lenVec1 = [positions[2][0]-positions[1][0], positions[2][1]-positions[1][1], positions[2][2]-positions[1][2]]
        lenVec1 = [cylEndPoint_1[0] - cylStartPoint_1[0], cylEndPoint_1[1] - cylStartPoint_1[1],
                   cylEndPoint_1[2] - cylStartPoint_1[2]]

        cylStartPoint_2 = [origin2.x + h1 * zAxis2.x, origin2.y + h1 * zAxis2.y, origin2.z + h1 * zAxis2.z]
        cylEndPoint_2 = [origin3.x + h1 * zAxis3.x, origin3.y + h1 * zAxis3.y, origin3.z + h1 * zAxis3.z]
        # lenVec2 = [positions[3][0]-positions[2][0], positions[3][1]-positions[2][1], positions[3][2]-positions[2][2]]
        lenVec2 = [cylEndPoint_2[0] - cylStartPoint_2[0], cylEndPoint_2[1] - cylStartPoint_2[1],
                   cylEndPoint_2[2] - cylStartPoint_2[2]]

        c2 = 0.7
        cylStartPoint_3 = [origin3.x + h1 * zAxis3.x, origin3.y + h1 * zAxis3.y, origin3.z + h1 * zAxis3.z]
        lenVec3 = [(lenVec2[0] - 0.5 * lenVec1[0]) * c2, (lenVec2[1] - 0.5 * lenVec1[1]) * c2,
                   (lenVec2[2] - 0.5 * lenVec1[2]) * c2]
        cylEndPoint_3 = [cylStartPoint_3[0] + lenVec3[0], cylStartPoint_3[1] + lenVec3[1],
                         cylStartPoint_3[2] + lenVec3[2]]
        # cylEndPoint_3 = [origin4.x + h1*zAxis4.x, origin4.y + h1*zAxis4.y, origin4.z + h1*zAxis4.z]
        # lenVec3 = [positions[4][0]-positions[3][0], positions[4][1]-positions[3][1], positions[4][2]-positions[3][2]]
        lenVec3 = [cylEndPoint_3[0] - cylStartPoint_3[0], cylEndPoint_3[1] - cylStartPoint_3[1],
                   cylEndPoint_3[2] - cylStartPoint_3[2]]


        cylStartPoint_6 = [origin4.x + h1 * zAxis4.x, origin4.y + h1 * zAxis4.y, origin4.z + h1 * zAxis4.z]
        cylEndPoint_6 = [origin5.x + h1 * zAxis5.x, origin5.y + h1 * zAxis5.y, origin5.z + h1 * zAxis5.z]
        # lenVec4 = [positions[5][0]-positions[4][0], positions[5][1]-positions[4][1], positions[5][2]-positions[4][2]]
        lenVec6 = [cylEndPoint_6[0] - cylStartPoint_6[0], cylEndPoint_6[1] - cylStartPoint_6[1],
                   cylEndPoint_6[2] - cylStartPoint_6[2]]

        cylStartPoint_7 = [origin5.x + h1 * zAxis5.x, origin5.y + h1 * zAxis5.y, origin5.z + h1 * zAxis5.z]
        cylEndPoint_7 = [origin6.x + h1 * zAxis6.x, origin6.y + h1 * zAxis6.y, origin6.z + h1 * zAxis6.z]
        # lenVec5 = [positions[6][0]-positions[5][0], positions[6][1]-positions[5][1], positions[6][2]-positions[5][2]]
        lenVec7 = [cylEndPoint_7[0] - cylStartPoint_7[0], cylEndPoint_7[1] - cylStartPoint_7[1],
                   cylEndPoint_7[2] - cylStartPoint_7[2]]

        c1 = 1
        cylStartPoint_4 = cylEndPoint_3
        _vec = [(-lenVec6[0] + 0.5 * lenVec7[0]) * c1, (-lenVec6[1] + 0.5 * lenVec7[1]) * c1,
                (-lenVec6[2] + 0.5 * lenVec7[2]) * c1]
        cylEndPoint_4 = [cylStartPoint_6[0] + _vec[0], cylStartPoint_6[1] + _vec[1], cylStartPoint_6[2] + _vec[2]]
        lenVec4 = [cylEndPoint_4[0] - cylStartPoint_4[0], cylEndPoint_4[1] - cylStartPoint_4[1],
                   cylEndPoint_4[2] - cylStartPoint_4[2]]

        cylStartPoint_5 = cylEndPoint_4
        cylEndPoint_5 = cylStartPoint_6
        lenVec5 = [cylEndPoint_5[0] - cylStartPoint_5[0], cylEndPoint_5[1] - cylStartPoint_5[1],
                   cylEndPoint_5[2] - cylStartPoint_5[2]]

        ballBody8 = generateBall(r1, rootComp).item(0)
        ballBody8 = ballBody8.createComponent()
        ballCom8 = ballBody8.parentComponent
        ballCom8.name = "Ball8"
        ballOcc8 = rootComp.allOccurrences.item(6)
        ballT8 = getT(cylEndPoint_7, [0.0, 0.0, 1.0])
        ballOcc8.transform2 = ballT8
        time.sleep(1.5)

        ballBody1 = generateBall(r1, rootComp).item(0)
        ballBody1 = ballBody1.createComponent()
        ballCom1 = ballBody1.parentComponent
        ballCom1.name = "Ball1"
        ballOcc1 = rootComp.allOccurrences.item(7)
        ballT1 = getT(cylStartPoint_4, [0.0, 0.0, 1.0])
        ballOcc1.transform2 = ballT1
        time.sleep(1.5)

        ballBody2 = generateBall(r1, rootComp).item(0)
        ballBody2 = ballBody2.createComponent()
        ballCom2 = ballBody2.parentComponent
        ballCom2.name = "Ball2"
        ballOcc2 = rootComp.allOccurrences.item(8)
        ballT2 = getT(cylStartPoint_5, [0.0, 0.0, 1.0])
        ballOcc2.transform2 = ballT2
        time.sleep(1.5)

        ballBody3 = generateBall(r1, rootComp).item(0)
        ballBody3 = ballBody3.createComponent()
        ballCom3 = ballBody3.parentComponent
        ballCom3.name = "Ball3"
        ballOcc3 = rootComp.allOccurrences.item(9)
        ballT3 = getT(cylStartPoint_3, [0.0, 0.0, 1.0])
        ballOcc3.transform2 = ballT3
        time.sleep(1.5)

        # ballBody4 = generateBall(r1, rootComp).item(0)
        # ballBody4 = ballBody4.createComponent()
        # ballCom4 = ballBody4.parentComponent
        # ballCom4.name = "Ball4"
        # ballOcc4 = rootComp.allOccurrences.item(16)
        # ballT4 = getT(cylStartPoint_4, [0.0, 0.0, 1.0])
        # ballOcc4.transform2 = ballT4
        #
        # ballBody5 = generateBall(r1, rootComp).item(0)
        # ballBody5 = ballBody5.createComponent()
        # ballCom5 = ballBody5.parentComponent
        # ballCom5.name = "Ball5"
        # ballOcc5 = rootComp.allOccurrences.item(17)
        # ballT5 = getT(cylStartPoint_5, [0.0, 0.0, 1.0])
        # ballOcc5.transform2 = ballT5
        #
        # ballBody6 = generateBall(r1, rootComp).item(0)
        # ballBody6 = ballBody6.createComponent()
        # ballCom6 = ballBody6.parentComponent
        # ballCom6.name = "Ball6"
        # ballOcc6 = rootComp.allOccurrences.item(18)
        # ballT6 = getT(cylStartPoint_6, [0.0, 0.0, 1.0])
        # ballOcc6.transform2 = ballT6
        #
        # ballBody7 = generateBall(r1, rootComp).item(0)
        # ballBody7 = ballBody7.createComponent()
        # ballCom7 = ballBody7.parentComponent
        # ballCom7.name = "Ball7"
        # ballOcc7 = rootComp.allOccurrences.item(19)
        # ballT7 = getT(cylStartPoint_7, [0.0, 0.0, 1.0])
        # ballOcc7.transform2 = ballT7

        cylStartPoint_1 = [origin1.x + h1*zAxis1.x, origin1.y + h1*zAxis1.y, origin1.z + h1*zAxis1.z]
        cylEndPoint_1 = [origin2.x + h1*zAxis2.x, origin2.y + h1*zAxis2.y, origin2.z + h1*zAxis2.z]
        # lenVec1 = [positions[2][0]-positions[1][0], positions[2][1]-positions[1][1], positions[2][2]-positions[1][2]]
        lenVec1 = [cylEndPoint_1[0]-cylStartPoint_1[0], cylEndPoint_1[1]-cylStartPoint_1[1], cylEndPoint_1[2]-cylStartPoint_1[2]]
        cylinderBody1 = generateCylinder(r1, getNorm(lenVec1), rootComp).item(0)
        cylinderBody1 = cylinderBody1.createComponent()
        cylinderCom1 = cylinderBody1.parentComponent
        cylinderCom1.name = "Cylinder1"
        cylinderOcc1 = rootComp.allOccurrences.item(10)
        cylinderT1 = getT(cylStartPoint_1, lenVec1)
        cylinderOcc1.transform2 = cylinderT1
        time.sleep(1.5)

        cylStartPoint_2 = [origin2.x + h1*zAxis2.x, origin2.y + h1*zAxis2.y, origin2.z + h1*zAxis2.z]
        cylEndPoint_2 = [origin3.x + h1*zAxis3.x, origin3.y + h1*zAxis3.y, origin3.z + h1*zAxis3.z]
        # lenVec2 = [positions[3][0]-positions[2][0], positions[3][1]-positions[2][1], positions[3][2]-positions[2][2]]
        lenVec2 = [cylEndPoint_2[0]-cylStartPoint_2[0], cylEndPoint_2[1]-cylStartPoint_2[1], cylEndPoint_2[2]-cylStartPoint_2[2]]
        cylinderBody2 = generateCylinder(r1, getNorm(lenVec2), rootComp).item(0)
        cylinderBody2 = cylinderBody2.createComponent()
        cylinderCom2 = cylinderBody2.parentComponent
        cylinderCom2.name = "Cylinder2"
        cylinderOcc2 = rootComp.allOccurrences.item(11)
        cylStartPoint_2 = [origin2.x + h1*zAxis2.x, origin2.y + h1*zAxis2.y, origin2.z + h1*zAxis2.z]
        cylinderT2 = getT(cylStartPoint_2, lenVec2)
        cylinderOcc2.transform2 = cylinderT2
        time.sleep(1.5)

        c2=0.7
        cylStartPoint_3 = [origin3.x + h1*zAxis3.x, origin3.y + h1*zAxis3.y, origin3.z + h1*zAxis3.z]
        lenVec3 = [(lenVec2[0]-0.5*lenVec1[0])*c2, (lenVec2[1]-0.5*lenVec1[1])*c2, (lenVec2[2]-0.5*lenVec1[2])*c2]
        cylEndPoint_3 = [cylStartPoint_3[0]+lenVec3[0], cylStartPoint_3[1]+lenVec3[1], cylStartPoint_3[2]+lenVec3[2]]
        # cylEndPoint_3 = [origin4.x + h1*zAxis4.x, origin4.y + h1*zAxis4.y, origin4.z + h1*zAxis4.z]
        # lenVec3 = [positions[4][0]-positions[3][0], positions[4][1]-positions[3][1], positions[4][2]-positions[3][2]]
        lenVec3 = [cylEndPoint_3[0]-cylStartPoint_3[0], cylEndPoint_3[1]-cylStartPoint_3[1], cylEndPoint_3[2]-cylStartPoint_3[2]]
        cylinderBody3 = generateCylinder(r1, getNorm(lenVec3), rootComp).item(0)
        cylinderBody3 = cylinderBody3.createComponent()
        cylinderCom3 = cylinderBody3.parentComponent
        cylinderCom3.name = "Cylinder3"
        cylinderOcc3 = rootComp.allOccurrences.item(12)
        cylStartPoint_3 = [origin3.x + h1*zAxis3.x, origin3.y + h1*zAxis3.y, origin3.z + h1*zAxis3.z]
        cylinderT3 = getT(cylStartPoint_3, lenVec3)
        cylinderOcc3.transform2 = cylinderT3
        time.sleep(1.5)

        c1=1
        cylStartPoint_4 = cylEndPoint_3
        _vec = [(-lenVec6[0]+0.5*lenVec7[0])*c1, (-lenVec6[1]+0.5*lenVec7[1])*c1, (-lenVec6[2]+0.5*lenVec7[2])*c1]
        cylEndPoint_4 = [cylStartPoint_6[0]+_vec[0], cylStartPoint_6[1]+_vec[1], cylStartPoint_6[2]+_vec[2]]
        lenVec4 = [cylEndPoint_4[0]-cylStartPoint_4[0], cylEndPoint_4[1]-cylStartPoint_4[1], cylEndPoint_4[2]-cylStartPoint_4[2]]
        cylinderBody4 = generateCylinder(r1, getNorm(lenVec4), rootComp).item(0)
        cylinderBody4 = cylinderBody4.createComponent()
        cylinderCom4 = cylinderBody4.parentComponent
        cylinderCom4.name = "Cylinder4"
        cylinderOcc4 = rootComp.allOccurrences.item(13)
        cylinderT4 = getT(cylStartPoint_4, lenVec4)
        cylinderOcc4.transform2 = cylinderT4
        time.sleep(1.5)

        cylStartPoint_5 = cylEndPoint_4
        cylEndPoint_5 = cylStartPoint_6
        lenVec5 = [cylEndPoint_5[0]-cylStartPoint_5[0], cylEndPoint_5[1]-cylStartPoint_5[1], cylEndPoint_5[2]-cylStartPoint_5[2]]
        cylinderBody5 = generateCylinder(r1, getNorm(lenVec5), rootComp).item(0)
        cylinderBody5 = cylinderBody5.createComponent()
        cylinderCom5 = cylinderBody5.parentComponent
        cylinderCom5.name = "Cylinder5"
        cylinderOcc5 = rootComp.allOccurrences.item(14)
        cylinderT5 = getT(cylStartPoint_5, lenVec5)
        cylinderOcc5.transform2 = cylinderT5
        time.sleep(1.5)

        cylStartPoint_6 = [origin4.x + h1*zAxis4.x, origin4.y + h1*zAxis4.y, origin4.z + h1*zAxis4.z]
        cylEndPoint_6 = [origin5.x + h1*zAxis5.x, origin5.y + h1*zAxis5.y, origin5.z + h1*zAxis5.z]
        # lenVec4 = [positions[5][0]-positions[4][0], positions[5][1]-positions[4][1], positions[5][2]-positions[4][2]]
        lenVec6 = [cylEndPoint_6[0]-cylStartPoint_6[0], cylEndPoint_6[1]-cylStartPoint_6[1], cylEndPoint_6[2]-cylStartPoint_6[2]]
        cylinderBody6 = generateCylinder(r1, getNorm(lenVec6), rootComp).item(0)
        cylinderBody6 = cylinderBody6.createComponent()
        cylinderCom6 = cylinderBody6.parentComponent
        cylinderCom6.name = "Cylinder6"
        cylinderOcc6 = rootComp.allOccurrences.item(15)
        cylinderT6 = getT(cylStartPoint_6, lenVec6)
        cylinderOcc6.transform2 = cylinderT6
        time.sleep(1.5)

        cylStartPoint_7 = [origin5.x + h1*zAxis5.x, origin5.y + h1*zAxis5.y, origin5.z + h1*zAxis5.z]
        cylEndPoint_7 = [origin6.x + h1*zAxis6.x, origin6.y + h1*zAxis6.y, origin6.z + h1*zAxis6.z]
        # lenVec5 = [positions[6][0]-positions[5][0], positions[6][1]-positions[5][1], positions[6][2]-positions[5][2]]
        lenVec7 = [cylEndPoint_7[0]-cylStartPoint_7[0], cylEndPoint_7[1]-cylStartPoint_7[1], cylEndPoint_7[2]-cylStartPoint_7[2]]
        cylinderBody7 = generateCylinder(r1, getNorm(lenVec7), rootComp).item(0)
        cylinderBody7 = cylinderBody7.createComponent()
        cylinderCom7 = cylinderBody7.parentComponent
        cylinderCom7.name = "Cylinder7"
        cylinderOcc7 = rootComp.allOccurrences.item(16)
        cylinderT7 = getT(cylStartPoint_7, lenVec7)
        cylinderOcc7.transform2 = cylinderT7
        time.sleep(1.5)



    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))



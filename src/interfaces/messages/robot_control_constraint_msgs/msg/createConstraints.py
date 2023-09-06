#!/usr/bin/python



types = ['Circle', 'Cone', 'Cylinder', 'Line', 'Plane', 'Point']

fileList = []

constraintsFile = "FixedConstraint[] fixed_constraints\n"

for type1 in types:
    for type2 in types:
        fileName = type1 + type2 + "Constraint.msg"
        fileList.append(fileName)
        content = "string id\n"
        content += type2 + " fixed\n"
        content += type1 + " constrained\n"
        content += "int32 type # ConstraintType\n"
        content += "bool flip\n"
        content += "float32[] parameter\n"
        content += "bool converged\n"
        content += "int32 fixedPoseIndex\n"
        content += "int32 constrainedPoseIndex\n"

        constraintsFile += type1 + type2 + "Constraint[] " + type1.lower() + "_" + type2.lower() + "_constraints\n"

        f = open(fileName,'w')
        f.write(content)
        f.close()

f = open("Constraints.msg",'w')
f.write(constraintsFile)
f.close()

print "Generated:\n" + "\n".join(fileList)

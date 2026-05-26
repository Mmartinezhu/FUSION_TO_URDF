# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: mmartinezhu
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils


def _link_name(occurrence):
    if occurrence.component.name == 'base_link':
        return 'base_link'
    return re.sub('[ :()]', '_', occurrence.name)


def _is_export_artifact(name):
    return name.startswith('old_component')


def _is_export_artifact_occurrence(occurrence, link_name):
    return _is_export_artifact(link_name) or occurrence.component.name.startswith('old_component')


def _reverse_joint_motion(joint_dict):
    if joint_dict['type'] in ['revolute', 'continuous', 'prismatic']:
        joint_dict['axis'] = [round(-i, 6) for i in joint_dict['axis']]

    if joint_dict['type'] in ['revolute', 'prismatic']:
        old_lower = joint_dict['lower_limit']
        old_upper = joint_dict['upper_limit']
        joint_dict['lower_limit'] = round(-old_upper, 6)
        joint_dict['upper_limit'] = round(-old_lower, 6)


def _add_oriented_joint(oriented_joints, entry, parent, child, reverse_motion):
    joint_dict = entry['data'].copy()
    if reverse_motion:
        _reverse_joint_motion(joint_dict)
    joint_dict['parent'] = parent
    joint_dict['child'] = child
    oriented_joints[entry['name']] = joint_dict


class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.ros2_control_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_ros2_control_xml(self):
        """
        Generate the ros2_control joint XML and hold it by self.ros2_control_xml.
        """

        joint = Element('joint')
        joint.attrib = {'name':self.name}

        command_interface = SubElement(joint, 'command_interface')
        command_interface.attrib = {'name':'position'}
        if self.type == 'revolute' or self.type == 'prismatic':
            lower = SubElement(command_interface, 'param')
            lower.attrib = {'name':'min'}
            lower.text = str(self.lower_limit)
            upper = SubElement(command_interface, 'param')
            upper.attrib = {'name':'max'}
            upper.text = str(self.upper_limit)

        for interface_name in ['position', 'velocity', 'effort']:
            state_interface = SubElement(joint, 'state_interface')
            state_interface.attrib = {'name':interface_name}

        self.ros2_control_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Backward-compatible alias for callers that still use the old method name.
        """

        self.make_ros2_control_xml()
        self.tran_xml = self.ros2_control_xml


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints
    
    
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
        
    Returns
    ----------
    joints_dict: 
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    success_msg = msg

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    joint_entries = []
    
    for joint in root.joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type
        
        # swhich by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0
        
        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            joint_dict['axis'] = [round(i, 6) for i in \
                joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'
                
        elif joint_type == 'prismatic':
            joint_dict['axis'] = [round(i, 6) for i in \
                joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled            
            if max_enabled and min_enabled:  
                joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
        elif joint_type == 'fixed':
            pass
        
        occurrence_one_name = _link_name(joint.occurrenceOne)
        occurrence_two_name = _link_name(joint.occurrenceTwo)

        if (_is_export_artifact_occurrence(joint.occurrenceOne, occurrence_one_name) or
                _is_export_artifact_occurrence(joint.occurrenceTwo, occurrence_two_name)):
            msg = ("El diseno de Fusion contiene componentes temporales llamados old_component. "
                   "Eso ocurre cuando una ejecucion anterior del exportador dejo componentes renombrados. "
                   "Deshaz esa ejecucion o reabre una copia limpia del diseno y vuelve a exportar.")
            break
        
        
        #There seem to be a problem with geometryOrOriginTwo. To calcualte the correct orogin of the generated stl files following approach was used.
        #https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
        #Thanks to Masaki Yamamoto!
        
        # Coordinate transformation by matrix
        # M: 4x4 transformation matrix
        # a: 3D vector
        def trans(M, a):
            ex = [M[0],M[4],M[8]]
            ey = [M[1],M[5],M[9]]
            ez = [M[2],M[6],M[10]]
            oo = [M[3],M[7],M[11]]
            b = [0, 0, 0]
            for i in range(3):
                b[i] = a[0]*ex[i]+a[1]*ey[i]+a[2]*ez[i]+oo[i]
            return(b)


        # Returns True if two arrays are element-wise equal within a tolerance
        def allclose(v1, v2, tol=1e-6):
            return( max([abs(a-b) for a,b in zip(v1, v2)]) < tol )

        try:
            xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray() # Relative Joint pos
            xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray() # Relative Joint pos
            xyz_of_one            = joint.occurrenceOne.transform.translation.asArray() # Link origin
            xyz_of_two            = joint.occurrenceTwo.transform.translation.asArray() # Link origin
            M_two = joint.occurrenceTwo.transform.asArray() # Matrix as a 16 element array.

        # Compose joint position
            case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
            case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
            if case1 or case2:
                xyz_of_joint = xyz_from_two_to_joint
            else:
                xyz_of_joint = trans(M_two, xyz_from_two_to_joint)


            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint]  # converted to meter

        except:
            try:
                if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                    data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                else:
                    data = joint.geometryOrOriginTwo.origin.asArray()
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
            except:
                msg = joint.name + " doesn't have joint origin. Please set it and run again."
                break
        
        joint_entries.append({
            'name': joint.name,
            'one': occurrence_one_name,
            'two': occurrence_two_name,
            'data': joint_dict
        })

    if msg != success_msg:
        return {}, msg

    joints_dict = {}
    visited_links = set(['base_link'])
    pending = joint_entries[:]

    while pending:
        progress = False
        next_pending = []

        for entry in pending:
            one = entry['one']
            two = entry['two']

            if one in visited_links and two not in visited_links:
                _add_oriented_joint(joints_dict, entry, one, two, True)
                visited_links.add(two)
                progress = True
            elif two in visited_links and one not in visited_links:
                _add_oriented_joint(joints_dict, entry, two, one, False)
                visited_links.add(one)
                progress = True
            elif one in visited_links and two in visited_links:
                msg = ("El modelo tiene un lazo cerrado entre {} y {}. URDF necesita una estructura de arbol; "
                       "convierte una de esas conexiones en fixed o elimina el ciclo antes de exportar.").format(one, two)
                return {}, msg
            else:
                next_pending.append(entry)

        if not progress:
            disconnected = ', '.join([entry['name'] for entry in next_pending])
            msg = ("No pude conectar todas las juntas con base_link. Juntas desconectadas: {}. "
                   "Verifica que exista un componente llamado base_link y que todas las juntas formen un arbol.").format(disconnected)
            return {}, msg

        pending = next_pending

    return joints_dict, msg

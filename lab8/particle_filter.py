from grid import *
from particle import Particle
from utils import *
from setting import *
from numpy.random import choice
import numpy
import scipy.stats
import time


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    dx = odom[0]
    dy = odom[1]
    dh = odom[2]

    motion_particles = []
    for particle in particles:
        newX = particle.x + dx * math.cos(math.radians(particle.h)) - dy * math.sin(math.radians(particle.h))
        newY = particle.y + dy * math.cos(math.radians(particle.h)) + dx * math.sin(math.radians(particle.h))
        newH = normalizeAngle(particle.h + dh)
        newParticle = Particle(newX, newY, newH)
        motion_particles.append(newParticle)
        
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """

    particlesSize = len(particles)

    #Remove particles that are out of bounds
    #particles = list(filter(lambda p: grid.is_in(p.x, p.y), particles))

    
    # Get the probability for each particle
    particleProbabilities = []
    for p in particles:
        
        #Handle the case where there are no measured markers by adding a random one
        #We just create a random particle and convert it to a measured marker
        if len(measured_marker_list) == 0:
            newP = Particle.create_random(7, grid)
            measured_marker_list = [(newP[0].x, newP[0].y, newP[0].h)]

        measuredMarkersProbProduct = 1
        for measuredMarker in measured_marker_list:
            probSum = 0

            for marker in grid.markers:
                probSum += getGaussianProb(measuredMarker, marker, p, grid)
            measuredMarkersProbProduct *= probSum

        # We can try alos something simpler
        # measureMarkersLength = len(measured_marker_list)
        # markersInCameraLength = len(p.read_markers(grid))
        # if measureMarkersLength == markersInCameraLength:
        #     measuredMarkersProbProduct = 1
        # else:
        #     measuredMarkersProbProduct = float(1) / (math.fabs(measureMarkersLength - markersInCameraLength) + 1)

        particleProbabilities.append(measuredMarkersProbProduct)

    #Normalize probabilities to add up to 1
    normalizedProbabilities = [float(prob) / sum(particleProbabilities) for prob in particleProbabilities]
    # Resample the particles depending on probability
    measured_particles = choice(particles, particlesSize, p=normalizedProbabilities)

    return measured_particles

def getGaussianProb(measuredMarker, marker, p, grid):
    """ Determines the probability of measuring marker given that particle p is the robot 

        Arguments: 
        measuredMarker -- The measured marker by robot
        marker -- The actual marker in the grid
        p -- The particle that estimates the robot's position
        grid -- The grid where the particle and markers are positioned

        Returns: the probability that p is actually measuring marker
    """

    sigma = 2

    # Get particle's coordinates on the map
    xP = p.x
    yP = p.y
    hP = p.h

    # Get the map coordinates of the marker
    m_xMap, m_yMap, m_hMap = parse_marker_info(marker[0], marker[1], marker[2])

    # Convert the marker's coordinates into particle's coordinates
    m_x = (m_xMap - xP) * math.cos(math.radians(hP)) + (m_yMap - yP) * math.sin(math.radians(hP))
    m_y = - (m_xMap - xP) * math.sin(math.radians(hP)) + (m_yMap - yP) * math.cos(math.radians(hP))
    m_h = diff_heading_deg(m_hMap, hP)

    probX = scipy.stats.norm.pdf(measuredMarker[0], loc=m_x, scale=sigma)
    probY = scipy.stats.norm.pdf(measuredMarker[1], loc=m_y, scale=sigma)
    probH = scipy.stats.norm.pdf(math.radians(measuredMarker[2]), loc=math.radians(m_h), scale=sigma)

    return probX * probY * probH


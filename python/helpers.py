from math import sqrt, exp
from matplotlib import pyplot as plt

class Vehicle(object):
    """
    Helper class. Non-ego vehicles move w/ constant acceleration
    """
    #input: start = [s, s_dot, s_dot_dot, d, d_dot, d_dot_dot]
    def __init__(self, start):
        self.start_state = start

    """
    Predict and returns the state of the vehicle at given timestep
    """
    def state_in(self, t):
        #extract s-coordinate value, velocity, acceleration=[s, s_dot, s_dot_dot]
        s = self.start_state[:3]
        #similarly for d-coordinate
        d = self.start_state[3:]

        #predict next state using eqs of motion for distance, velocity and acceleration
        state = [
            #using equation of motion for s-coordinate
            #S_t+1 = S_t + Vt * t + (a * t^2)/2
            s[0] + (s[1] * t) + s[2] * t**2 / 2.0,
            #as v = v + a * t
            s[1] + s[2] * t,
            #acceleration is constant so it remains
            s[2],
            #using equation of motion for d-coordinate
            #d_t+1 = d_t + Vt * t + (a * t^2)/2
            d[0] + (d[1] * t) + d[2] * t**2 / 2.0,
            #as v = a * t
            d[1] + d[2] * t,
            #acceleration is constant so it remains
            d[2],
        ]
        return state

#-------------End of class Vehicle------------#

"""
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

Useful for cost functions.
"""
def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + exp(-x)) - 1.0

"""
Takes the coefficients of a polynomial and creates a function of
time from them and returns a reference to that function for later use
"""
def to_equation(coefficients):
    """
    Takes the coefficients of a polynomial and creates a function of
    time from them.
    """
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients):
            total += c * t ** i
        return total
    return f

"""
Calculates the derivative of a polynomial and returns
the corresponding coefficients.
"""
def differentiate(coefficients):
    """
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
    """

    new_cos = []
    #ignore the first coefficient as it will always become 0
    #when taking derivative of a polynomial
    for deg, prev_co in enumerate(coefficients[1:]):
        #use (deg+1) because we have ignore first coefficient but
        #array indexing still starts from 0
        #so a^2 will become 2a
        new_cos.append((deg+1) * prev_co)

    #return update coefficients
    return new_cos

"""
Calculates nearest approach with given `vehicles` during a given trajectory `traj`
of ego vehicle
"""
def nearest_approach_to_any_vehicle(traj, vehicles):
    """
    Calculates the closest distance to any vehicle during a trajectory.
    """
    closest = 999999
    #for each of other vehicles, keep track of min distance to any vehicle
    #at any timestep in [0, traj.T]
    for v in vehicles.values():
        #get min distance for any timestep in rang: [0, traj.T]
        #between ego vehicles' trajector `traj` and other vehicle `v`
        d = nearest_approach(traj,v)

        #only keep track of min distance to any vehicle at any timestep
        if d < closest:
            closest = d
    return closest

"""
Calculates nearest approach with given `vehicle` during a given trajectory `traj`
of ego vehicle
"""
def nearest_approach(traj, vehicle):
    closest = 999999
    #extract all (s, d) values and total time T for given trajectory and
    s_,d_,T = traj
    #make a polynomial function of time with s-values `s_` as coefficients
    #Note: this is a function of time so will be called like: s(t)
    s = to_equation(s_)
    #make a polynomial function of time with d-values `d_` as coefficients
    #Note: this is a function of time so will be called like: d(t)
    d = to_equation(d_)

    #divide total time T in 100 timesteps and
    #update the state of vehicle for each timestep
    for i in range(100):
        #get (i%) of total time
        # `i` in this equation is actually telling the percentage of time
        # so for example i=2 means 2% of total time `T`.
        t = float(i) / 100 * T
        #get s-coordinate for given trajectory
        # at timestep `t` using `s(t)`polynomial function of time
        cur_s = s(t)
        #get d-coordinate for given trajectory
        #at timestep `t` using `d(t)`polynomial function of time
        cur_d = d(t)
        #predict the new state [s, s_dot, s_dot_dot, d, d_dot, d_dot_dot]
        #of other vehicle for time interval t
        targ_s, _, _, targ_d, _, _ = vehicle.state_in(t)
        #check the distance between other vehicle and ego vehicle
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        #update the closest approach distance
        if dist < closest:
            closest = dist
    return closest

"""
Plots the trajectory of ego_vehicle with given coefficients (s_coeffs, d_coeffs)
and other_vehicle (if given) for total time of T
"""
def show_trajectory(s_coeffs, d_coeffs, T, vehicle=None):
    #make a polynomial function of time with s-values `s_coeffs` as coefficients
    #Note: this is a function of time so will be called like: s(t)
    s = to_equation(s_coeffs)
    #make a polynomial function of time with d-values `d_coeffs` as coefficients
    #Note: this is a function of time so will be called like: d(t)
    d = to_equation(d_coeffs)

    #arrays to hold (s, d) values for trajectory
    X = []
    Y = []

    if vehicle:
        X2 = []
        Y2 = []

    t = 0
    #for each timestep of 0.25
    while t <= T+0.01:
        #predict (s, d) for ego vehicle at time `t`
        #using polynomial functions of time and append them to arrays
        X.append(s(t))
        Y.append(d(t))

        #predict state of vehicle at time `t` using eqs of motion
        #and append them to arrays
        if vehicle:
            s_, _, _, d_, _, _ = vehicle.state_in(t)
            X2.append(s_)
            Y2.append(d_)

        #go to next timestep
        t += 0.25

    #now that we have trajectory for both ego vehicle and other vehicle
    #plot them

    #plot ego vehicle trajectory in blue
    plt.scatter(X,Y,color="blue")

    #plot other vehicle trajectory in red
    if vehicle:
        plt.scatter(X2, Y2,color="red")

    plt.show()

"""
Function to make polynomial, take 3 time derivatives (each new derivative is a derivative of old one) of
 that polynomial, make a function of time for actual polynomial and 3 differentiated polynomialss
 and return the list of those functions
 output=[
    polynomial function of time to calculate `s` or `d` coordinate,
    1st derivate polynomial function of time to calculate speed of for `s` or `d`
    2nd derivate polynomial function of time to calculate acceleration for `s` or `d`
    3rd derivate polynomial function of time to calculate jerk for `s` or `d`
    ]
"""
def get_f_and_N_derivatives(coeffs, N=3):
    #make a polynomial function of time with passed coeffs
    functions = [to_equation(coeffs)]
    for i in range(N):
        #take ith derivative of coefficients
        coeffs = differentiate(coeffs)
        #make a function of time of ith time differentiated coefficients
        functions.append(to_equation(coeffs))

    #return the list of functions
    return functions

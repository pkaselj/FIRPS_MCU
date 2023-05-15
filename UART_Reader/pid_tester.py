from matplotlib import pyplot as plt

KP = 2.2049
TI = 1/128.8773
STEPS = 100
TIMESTEP = 0.016
SETPOINT = 0.5
A = 0.04924
B = 0.2617

class FirstOrderPlant:
    A : float
    B : float

    _x : float
    _y : float
    _x_1 : float
    _y_1 : float

    def __init__(self, A, B) -> None:
        self.A = A
        self.B = B

        self._x = 0
        self._x_1 = 0

        self._y = 0
        self._y_1 = 0
    
    def Advance(self, x : float) -> float:
        self._x_1 = self._x
        self._x = x

        self._y_1 = self._y

        self._y = self.A * self._x_1 + self.B * self._y_1
        return self._y


class PIController:
    Kp : float
    Ti : float
    out_max : float
    out_min : float

    _current_error : float
    _previous_error : float
    _current_output : float
    _previous_output : float

    def __init__(self, Kp, Ti, out_min, out_max) -> None:
        self.Kp = Kp
        self.Ti = Ti
        self.out_min = out_min
        self.out_max = out_max
        self._current_error = 0
        self._previous_error = 0
        self._current_output = 0
        self._previous_output = 0

    def CalculateSample(self, error : float, timestep : float) -> float:
        self._previous_error = self._current_error
        self._current_error = error

        self._previous_output = self._current_output

        current_error_term = self.Kp * (1 + timestep / self.Ti)
        previous_error_term = -self.Kp

        self._current_output = (
            current_error_term * self._current_error
            + previous_error_term * self._previous_error
            + self._current_output
        )

        if self._current_output > self.out_max:
            self._current_output = self.out_max
        elif self._current_output < self.out_min:
            self._current_output = self.out_min

        return self._current_output


if __name__ == '__main__':

    pid = PIController(KP, TI, 0, 95)
    plant = FirstOrderPlant(A, B)

    y = STEPS * [0,]
    x = STEPS * [0,]
    e = STEPS * [0,]

    for i in range(1, STEPS):
        e[i] = SETPOINT - y[i - 1]
        x[i] = pid.CalculateSample(e[i], TIMESTEP)
        y[i] = plant.Advance(x[i])

    t = range(STEPS)
    t = [v * TIMESTEP for v in t]

    plt.plot(t, y, 'b')
    plt.plot(t, x, 'g')
    plt.plot(t, e, 'r')
    plt.show()
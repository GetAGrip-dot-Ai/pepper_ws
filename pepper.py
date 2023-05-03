from pepper_fruit import PepperFruit
from pepper_peduncle import PepperPeduncle


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class for a matched pepper and its properties
"""


class Pepper:
    def __init__(self, number: int, pf_number: int, pp_number: int):
        self._number = number
        self._order: int = -1
        self._pepper_fruit: PepperFruit = PepperFruit(pf_number)
        self._pepper_peduncle: PepperPeduncle = PepperPeduncle(pp_number)

    @property
    def number(self):
        return self._number

    @property
    def order(self):
        return self._order

    @order.setter
    def order(self, order):
        self._order = order

    @property
    def pepper_fruit(self):
        return self._pepper_fruit

    @pepper_fruit.setter
    def pepper_fruit(self, value):
        self._pepper_fruit = value

    @property
    def pepper_peduncle(self):
        return self._pepper_peduncle

    @pepper_peduncle.setter
    def pepper_peduncle(self, value):
        self._pepper_peduncle = value

    def __str__(self):
        return f"Pepper #{self.number}"

from pepper_fruit import PepperFruit
from one_frame import OneFrame
from multi_frame import MultiFrame

if __name__ == '__main__':
    obj = MultiFrame()

    f1 = OneFrame("")
    f2 = OneFrame("")
    f3 = OneFrame("")

    f1.add_detected_pepper_fruit(PepperFruit(0, [20,20,10,10], 0.8))
    f1.add_detected_pepper_fruit(PepperFruit(1, [50,50,10,10], 0.8))

    f2.add_detected_pepper_fruit(PepperFruit(2, [22,22,10,10], 0.8))
    # f2.add_detected_pepper_fruit(PepperFruit(3, [47,47,10,10], 0.8))

    f3.add_detected_pepper_fruit(PepperFruit(4, [21,24,10,10], 0.8))

    obj.add_one_frame(f1)
    obj.add_one_frame(f2)
    obj.add_one_frame(f3)

    # obj.populate_frames()

    obj.find_fruit_true_positives()
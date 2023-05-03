import numpy as np
from pepper_fruit_utils import *
from pepper_peduncle_utils import *


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 8, 2023
Code description: Functions that perform different components of the multi-frame algorithm
"""


def get_best_fruit_match(fruit1, fruits2):
    x, y, w, h = fruit1.xywh
    box1 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]

    iou_list = []

    for fruit2 in fruits2:
        x, y, w, h = fruit2.xywh
        box2 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]

        iou = calculate_iou(box1, box2)
        iou_list.append((fruit2, iou))

    iou_list.sort(key=lambda x: x[1], reverse=True)

    return iou_list[0] if len(iou_list) > 0 else (None, 0) 


def update_fruit_occurences(fruits_frame1, fruits_frame2, frame1_number, frame2_number):
    for fruit1 in fruits_frame1:
        fruit2, max_iou = get_best_fruit_match(fruit1, fruits_frame2)

        if max_iou > 0.3:  # Need to tune
            fruit1.occurences += 1
            fruit2.occurences += 1

            fruit1.add_associated_fruit(frame2_number, fruit2.number)
            fruit2.add_associated_fruit(frame1_number, fruit1.number)
            # print(f"fruit 1: {fruit1.number}, fruit 2: {fruit2.number}")


def update_fruit_true_positives(fruits, max_frames):
    for fruit in fruits:
        if fruit.occurences >= 1:
            fruit.true_positive = True
    

def get_all_fruits(frames):
    all_frames_positive_fruits = []
    unique_positive_fruits = {}
    unmatched_positive_fruits = {}

    for i in range(len(frames)):
        frame = frames[i]
        
        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                all_frames_positive_fruits.append(fruit)

    for i in reversed(range(len(frames))):
        frame = frames[i]

        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                if fruit in all_frames_positive_fruits:

                    all_frames_positive_fruits.remove(fruit)
                    # print(all_frames_positive_fruits)
                    
                    if fruit.parent_pepper != None:
                        unique_positive_fruits[frame.frame_number] = unique_positive_fruits.get(frame.frame_number, []) + [fruit]

                        for frame_number, associated_fruit_number in fruit.associated_fruits:
                            associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                            if associated_fruit in all_frames_positive_fruits:
                                all_frames_positive_fruits.remove(associated_fruit)
                    else:
                        found = False
                        while not found:
                            for frame_number, associated_fruit_number in fruit.associated_fruits:
                                
                                associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                                if associated_fruit.parent_pepper != None:
                                    unique_positive_fruits[frame_number] = unique_positive_fruits.get(frame_number, []) + [associated_fruit]
                                    
                                    found = True
                                    break
                            
                            if not found:
                                unmatched_positive_fruits[frame.frame_number] = unmatched_positive_fruits.get(frame.frame_number, []) + [fruit]
                                # found = True
                                break

                        for frame_number, associated_fruit_number in fruit.associated_fruits:
                            associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                            if associated_fruit in all_frames_positive_fruits:
                                all_frames_positive_fruits.remove(associated_fruit)

    return unique_positive_fruits, unmatched_positive_fruits


def get_best_peduncle_match(peduncle1, peduncles2):
    iou_list = []

    for peduncle2 in peduncles2:
        iou = calculate_peduncle_iou(peduncle1.mask, peduncle2.mask)
        iou_list.append((peduncle2, iou))

    iou_list.sort(key=lambda x: x[1], reverse=True)

    return iou_list[0] if len(iou_list) > 0 else (None, 0) 


def update_peduncle_occurences(peduncles_frame1, peduncles_frame2, frame1_number, frame2_number):
    for peduncle1 in peduncles_frame1:
        peduncle2, max_iou = get_best_peduncle_match(peduncle1, peduncles_frame2)

        if max_iou > 0.3:  # Need to tune
            peduncle1.occurences += 1
            peduncle2.occurences += 1

            peduncle1.add_associated_peduncle(frame2_number, peduncle2.number)
            peduncle2.add_associated_peduncle(frame1_number, peduncle1.number)
            # print(f"peduncle 1: {peduncle1.number}, peduncle 2: {peduncle2.number}")


def update_peduncle_true_positives(peduncles, max_frames):
    for peduncle in peduncles:
        if peduncle.occurences >= 0.5*max_frames:
            peduncle.true_positive = True


def get_all_peduncles(frames):
    all_frames_positive_peduncles = []
    unique_positive_peduncles = {}
    unmatched_positive_peduncles = {}

    for i in range(len(frames)):
        frame = frames[i]
        
        for peduncle in frame.pepper_peduncle_detections.values():
            if peduncle.true_positive:
                all_frames_positive_peduncles.append(peduncle)

    for i in reversed(range(len(frames))):
        frame = frames[i]

        for peduncle in frame.pepper_peduncle_detections.values():
            if peduncle.true_positive:
                if peduncle in all_frames_positive_peduncles:

                    all_frames_positive_peduncles.remove(peduncle)
                    
                    if peduncle.parent_pepper != None:
                        unique_positive_peduncles[frame.frame_number] = unique_positive_peduncles.get(frame.frame_number, []) + [peduncle]

                        for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                            associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                            if associated_peduncle in all_frames_positive_peduncles:
                                all_frames_positive_peduncles.remove(associated_peduncle)
                    else:
                        found = False
                        while not found:
                            for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                                
                                associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                                if associated_peduncle.parent_pepper != None:
                                    unique_positive_peduncles[frame_number] = unique_positive_peduncles.get(frame_number, []) + [associated_peduncle]

                                    found = True
                                    break
                            
                            if not found:
                                unmatched_positive_peduncles[frame.frame_number] = unmatched_positive_peduncles.get(frame.frame_number, []) + [peduncle]
                                break

                        for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                            associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                            if associated_peduncle in all_frames_positive_peduncles:
                                all_frames_positive_peduncles.remove(associated_peduncle)
                
    return unique_positive_peduncles, unmatched_positive_peduncles


def get_image_webcam():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    return frame

def draw_bounding_box(ax, number, x, y, w, h, color="blue", fill=False):
    rect = patches.Rectangle((x - w / 2, y - h / 2), w, h, linewidth=2, edgecolor=color, facecolor='none')
    ax.add_patch(rect)
    ax.text(x - w / 2, y - h / 2, number, color=color, fontsize=10)


def draw_bounding_polygon(ax, mask, img_shape, color="blue", fill=True):
    mask = torch.transpose(mask, 0, 1)
    polygon = Polygon(mask.T)
    x, y = polygon.exterior.xy
    x_scaled = [i*img_shape[1] for i in x]
    y_scaled = [i*img_shape[0] for i in y]
    if fill:
        ax.fill(x_scaled, y_scaled, color=color, alpha=0.7)
    polygon = Polygon(mask.T)
    ax.plot(*polygon.exterior.xy)


def plot_frames(mf_obj, chosen_frame_num, chosen_fruit_num):
    try:
        frames = mf_obj._one_frames
        num_frames = len(frames)

        if num_frames > 1:
            fig, axs = plt.subplots(1, num_frames)  
        else:
            fig, axis = plt.subplots()
            axs = [axis]

        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
        plt.margins(0, 0)

        images = []
        for i in range(num_frames):
            images.append(np.asarray(Image.open(f"{os.getcwd()}/test_multi_frame/log/frame_{i}.png")))
            axs[i].imshow(images[i])
            axs[i].axis('off')

        if chosen_frame_num == -1:
            plt.title(f"No pepper chosen", fontsize=10)
            plt.savefig(os.getcwd() + '/test_multi_frame/log/all_frames.png', bbox_inches='tight')
            print(colored("All multi-frames plots have been saved", "blue"))
            plt.close()
            return
        else:
            plt.title(f"Chosen frame: {chosen_frame_num}, Chosen fruit: {chosen_fruit_num}", fontsize=10)
            

        for frame_number, peppers in mf_obj._matched_positive_peppers.items():

            for pepper in peppers:
                r = np.round(np.random.rand(), 1)
                g = np.round(np.random.rand(), 1)/5
                b = np.round(np.random.rand(), 1)
                color = (r, g, b)

                pepper_fruit = pepper.pepper_fruit
                xywh = pepper_fruit.xywh
                x = int(xywh[0])
                y = int(xywh[1])
                w = int(xywh[2])
                h = int(xywh[3])
                draw_bounding_box(axs[frame_number], pepper_fruit.number, x, y, w, h, color=color)

                pepper_peduncle = pepper.pepper_peduncle
                mask = pepper_peduncle.mask
                draw_bounding_polygon(axs[frame_number], mask, images[frame_number].shape, color=color)
                poi_px = pepper.pepper_peduncle.poi_px
                axs[frame_number].plot(poi_px[1], poi_px[0], 'ro', markersize=2)

                for associated_frame_number, associated_fruit_number in pepper_fruit.associated_fruits:
                    associated_fruit = frames[associated_frame_number].pepper_fruit_detections[associated_fruit_number]
                    xywh = associated_fruit.xywh
                    x = int(xywh[0])
                    y = int(xywh[1])
                    w = int(xywh[2])
                    h = int(xywh[3])
                    draw_bounding_box(axs[associated_frame_number], associated_fruit.number, x, y, w, h, color=color)

                    if associated_fruit.parent_pepper is not None:
                        parent_pepper = frames[associated_frame_number].pepper_detections[associated_fruit.parent_pepper]
                        associated_peduncle = parent_pepper.pepper_peduncle
                        mask = associated_peduncle.mask
                        draw_bounding_polygon(axs[associated_frame_number], mask, images[associated_frame_number].shape, color=color)
                        poi_px = associated_peduncle.poi_px
                        axs[associated_frame_number].plot(poi_px[1], poi_px[0], 'ro', markersize=2)
        
        for frame_number, fruits in mf_obj._unmatched_positive_fruits.items():

            for pepper_fruit in fruits:
                color = (0, 0, 0)

                xywh = pepper_fruit.xywh
                x = int(xywh[0])
                y = int(xywh[1])
                w = int(xywh[2])
                h = int(xywh[3])
                draw_bounding_box(axs[frame_number], pepper_fruit.number, x, y, w, h, color=color)

                for associated_frame_number, associated_fruit_number in pepper_fruit.associated_fruits:
                    associated_fruit = frames[associated_frame_number].pepper_fruit_detections[associated_fruit_number]
                    xywh = associated_fruit.xywh
                    # import ipdb; ipdb.set_trace();
                    x = int(xywh[0])
                    y = int(xywh[1])
                    w = int(xywh[2])
                    h = int(xywh[3])
                    draw_bounding_box(axs[associated_frame_number], associated_fruit.number, x, y, w, h, color=color)

                    if associated_fruit.parent_pepper is not None:
                        parent_pepper = frames[associated_frame_number].pepper_detections[associated_fruit.parent_pepper]
                        associated_peduncle = parent_pepper.pepper_peduncle
                        mask = associated_peduncle.mask
                        draw_bounding_polygon(axs[associated_frame_number], mask, images[associated_frame_number].shape, color=color)
                        poi_px = associated_peduncle.poi_px
                        axs[associated_frame_number].plot(poi_px[1], poi_px[0], 'ro', markersize=2)

        # for frame_number, peduncles in mf_obj._unmatched_positive_peduncles.items():
        #     color = (0, 0, 0)
            
        #     for pepper_peduncle in peduncles:
        #         mask = pepper_peduncle.mask
        #         draw_bounding_polygon(axs[frame_number], mask, images[frame_number].shape, color=color)
        #         poi_px = pepper_peduncle.poi_px
        #         axs[frame_number].plot(poi_px[1], poi_px[0], 'ro', markersize=2)

        plt.savefig(os.getcwd() + '/test_multi_frame/log/all_frames.png', bbox_inches='tight')
        plt.close()
        print(colored("All multi-frames plots have been saved", "blue"))
    except Exception as e:
        print(colored(f"Error occured while trying to save all multi-frames together: {e}", "red"))
        
    
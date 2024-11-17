# src/gui_interface.py
import customtkinter as ctk
from tkinter import messagebox, simpledialog
import subprocess
import json
import cv2
import numpy as np

class ManipulatorGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        # configure window
        self.title("Manipulator Control Interface")
        self.geometry(f"{800}x{600}")

        # initialize color_ranges
        self.color_ranges = {
            'red': ([0, 0, 88], [105, 28, 155]),
            'green': ([26, 40, 0], [75, 255, 31]),
            'purple': ([48, 0, 0], [85, 23, 67]),
            'orange': ([0, 35, 93], [38, 84, 155])
        }

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)

        # create sidebar frame with widgets
        self.sidebar_frame = ctk.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = ctk.CTkLabel(self.sidebar_frame, text="Manipulator GUI", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.workspace_calibration_button = ctk.CTkButton(self.sidebar_frame, corner_radius=0, height=40, border_spacing=10, text="Workspace Calibration",
                                                          fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                          anchor="w", command=self.workspace_calibration_event)
        self.workspace_calibration_button.grid(row=1, column=0, sticky="ew")

        self.define_color_masking_button = ctk.CTkButton(self.sidebar_frame, corner_radius=0, height=40, border_spacing=10, text="Define Color Masking",
                                                         fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                         anchor="w", command=self.define_color_masking_event)
        self.define_color_masking_button.grid(row=2, column=0, sticky="ew")

        self.controlling_manipulator_button = ctk.CTkButton(self.sidebar_frame, corner_radius=0, height=40, border_spacing=10, text="Controlling Manipulator",
                                                            fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                                            anchor="w", command=self.controlling_manipulator_event)
        self.controlling_manipulator_button.grid(row=3, column=0, sticky="ew")

        self.appearance_mode_label = ctk.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = ctk.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"],
                                                            command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.appearance_mode_optionemenu.set("Dark")

        # create frames for each section
        self.workspace_calibration_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.define_color_masking_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.controlling_manipulator_frame = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")

        # configure frames
        self.workspace_calibration_frame.grid_columnconfigure(0, weight=1)
        self.define_color_masking_frame.grid_columnconfigure(0, weight=1)
        self.controlling_manipulator_frame.grid_columnconfigure(0, weight=1)

        # add widgets to define_color_masking_frame
        self.color_listbox = ctk.CTkTextbox(self.define_color_masking_frame, height=10, width=30)
        self.color_listbox.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        self.update_color_listbox()

        # Create a frame to hold the buttons
        self.button_frame = ctk.CTkFrame(self.define_color_masking_frame, fg_color="transparent")
        self.button_frame.grid(row=1, column=0, columnspan=2, pady=10)

        self.add_color_button = ctk.CTkButton(self.button_frame, text="Add Color", command=self.add_color)
        self.add_color_button.grid(row=0, column=0, padx=(5, 40), pady=5)

        self.remove_color_button = ctk.CTkButton(self.button_frame, text="Remove Color", command=self.remove_color)
        self.remove_color_button.grid(row=0, column=1, padx=(40, 5), pady=5)

        # select default frame
        self.select_frame_by_name("workspace_calibration")

    def select_frame_by_name(self, name):
        # set button color for selected button
        self.workspace_calibration_button.configure(fg_color=("gray75", "gray25") if name == "workspace_calibration" else "transparent")
        self.define_color_masking_button.configure(fg_color=("gray75", "gray25") if name == "define_color_masking" else "transparent")
        self.controlling_manipulator_button.configure(fg_color=("gray75", "gray25") if name == "controlling_manipulator" else "transparent")

        # show selected frame
        if name == "workspace_calibration":
            self.workspace_calibration_frame.grid(row=0, column=1, columnspan=3, rowspan=4, sticky="nsew", padx=20, pady=20)
        else:
            self.workspace_calibration_frame.grid_forget()
        if name == "define_color_masking":
            self.define_color_masking_frame.grid(row=0, column=1, columnspan=3, rowspan=4, sticky="nsew", padx=20, pady=20)
        else:
            self.define_color_masking_frame.grid_forget()
        if name == "controlling_manipulator":
            self.controlling_manipulator_frame.grid(row=0, column=1, columnspan=3, rowspan=4, sticky="nsew", padx=20, pady=20)
        else:
            self.controlling_manipulator_frame.grid_forget()

    def workspace_calibration_event(self):
        self.select_frame_by_name("workspace_calibration")

    def define_color_masking_event(self):
        self.select_frame_by_name("define_color_masking")

    def controlling_manipulator_event(self):
        self.select_frame_by_name("controlling_manipulator")

    def update_color_listbox(self):
        self.color_listbox.delete("1.0", ctk.END)
        for color in self.color_ranges:
            self.color_listbox.insert(ctk.END, color + "\n")

    def add_color(self):
        color_name = simpledialog.askstring("Input", "Enter color name:")
        if color_name:
            def nothing(x):
                pass

            # Create a window
            cv2.namedWindow('Masked Image')

            # Create trackbars for lower bound color change
            cv2.createTrackbar('Lower R', 'Masked Image', 0, 255, nothing)
            cv2.createTrackbar('Lower G', 'Masked Image', 0, 255, nothing)
            cv2.createTrackbar('Lower B', 'Masked Image', 0, 255, nothing)

            # Create trackbars for upper bound color change
            cv2.createTrackbar('Upper R', 'Masked Image', 255, 255, nothing)
            cv2.createTrackbar('Upper G', 'Masked Image', 255, 255, nothing)
            cv2.createTrackbar('Upper B', 'Masked Image', 255, 255, nothing)

            # Capture video from the webcam
            for i in range(0, 10):
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    break
            else:
                print("Cannot open the webcam")
                return

            while True:
                # Read the frame
                ret, frame = cap.read()
                if not ret:
                    break

                # Get current positions of the trackbars for lower bound
                lower_r = cv2.getTrackbarPos('Lower R', 'Masked Image')
                lower_g = cv2.getTrackbarPos('Lower G', 'Masked Image')
                lower_b = cv2.getTrackbarPos('Lower B', 'Masked Image')

                # Get current positions of the trackbars for upper bound
                upper_r = cv2.getTrackbarPos('Upper R', 'Masked Image')
                upper_g = cv2.getTrackbarPos('Upper G', 'Masked Image')
                upper_b = cv2.getTrackbarPos('Upper B', 'Masked Image')

                # Define the lower and upper bounds for the color
                lower_bound = np.array([lower_r, lower_g, lower_b])
                upper_bound = np.array([upper_r, upper_g, upper_b])

                # Create a mask
                mask = cv2.inRange(frame, lower_bound, upper_bound)
                masked_image = cv2.bitwise_and(frame, frame, mask=mask)

                # Display the masked image
                cv2.imshow('Masked Image', masked_image)

                # Break the loop when 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Release the capture and destroy all windows
            cap.release()
            cv2.destroyAllWindows()

            # Save the selected color range
            self.color_ranges[color_name] = (lower_bound.tolist(), upper_bound.tolist())
            self.update_color_listbox()

    def remove_color(self):
        selected_color = self.color_listbox.get("1.0", ctk.END).strip()
        if selected_color in self.color_ranges:
            del self.color_ranges[selected_color]
            self.update_color_listbox()

    def calibrate_workspace(self):
        # Implement workspace calibration logic here
        messagebox.showinfo("Info", "Workspace calibration not implemented yet.")

    def launch_manipulator(self):
        # Launch the manipulator script
        messagebox.showinfo("Info", "Manipulator script launched not implemented yet.")

    def change_appearance_mode_event(self, new_appearance_mode: str):
        ctk.set_appearance_mode(new_appearance_mode)

if __name__ == "__main__":
    ctk.set_appearance_mode("dark")  # Modes: "System" (default), "Dark", "Light"
    ctk.set_default_color_theme("blue")  # Themes: "blue" (default), "green", "dark-blue"

    app = ManipulatorGUI()
    app.mainloop()
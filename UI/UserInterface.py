import customtkinter
import numpy as np

class pointEntriesFrame(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        self.grid_columnconfigure(0, minsize=200)
        self.grid_columnconfigure(1, minsize=100)
        self.combobox_value = "Rectangle"
        self.pointCoords = []
        self.areaData = []

        def update_point_entries(cleaningAreaType):
            if cleaningAreaType == "Rectangle":
                self.entry_2_title.configure(text="Right Edge")
                self.entry_2.delete(0, customtkinter.END)
                self.entry_2.insert(0, "2.8, 0.0")
                self.entry_3_title.configure(text="Top-Right Corner")
                self.entry_3.delete(0, customtkinter.END)
                self.entry_3.insert(0, "2.8, 3.0")
                self.entry_4_title.configure(text="Top-Left Corner")
                self.entry_4.delete(0, customtkinter.END)
                self.entry_4.insert(0, "0.0, 3.0")
                self.entry_5_title.configure(text="NA")
                self.entry_5.delete(0, customtkinter.END)
                self.entry_5.insert(0, "NA")
                self.entry_5.configure(state="disabled")
                self.entry_6_title.configure(text="NA")
                self.entry_6.delete(0, customtkinter.END)
                self.entry_6.insert(0, "NA")
                self.entry_6.configure(state="disabled")

            elif cleaningAreaType == "L-Shaped":
                self.entry_2_title.configure(text="Right Edge of Top")
                self.entry_2.delete(0, customtkinter.END)
                self.entry_2.insert(0, "4.8, 0.0")
                self.entry_3_title.configure(text="Top-Right Inner Corner")
                self.entry_3.delete(0, customtkinter.END)
                self.entry_3.insert(0, "4.8, 1.5")
                self.entry_4_title.configure(text="Bottom-Right Inner Corner")
                self.entry_4.delete(0, customtkinter.END)
                self.entry_4.insert(0, "1.5, 1.5")
                self.entry_5_title.configure(text="Top-Right Outer Corner")
                self.entry_5.configure(state="normal")
                self.entry_5.delete(0, customtkinter.END)
                self.entry_5.insert(0, "1.5, 4.3")
                self.entry_6_title.configure(text="Top-Left Corner")
                self.entry_6.configure(state="normal")
                self.entry_6.delete(0, customtkinter.END)
                self.entry_6.insert(0, "0.0, 4.3")

            else:
                print("No Cleaning Area Selected")

        def combobox_callback(choice):
            self.combobox_value = choice
            update_point_entries(choice)


        self.title = customtkinter.CTkLabel(self, text="Cleaning Area Shape", fg_color="grey50", corner_radius=6)
        self.title.grid(row=0, column=0, padx=10, pady=(10,0), sticky="ew")          

        self.combobox = customtkinter.CTkComboBox(self, values = ["Rectangle", "L-Shaped"], command=combobox_callback)
        self.combobox.set("Rectangle")
        self.combobox.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_1_title = customtkinter.CTkLabel(self, text="Starting Point", fg_color="grey50", corner_radius=6)
        self.entry_1_title.grid(row=1, column=0, padx=10, pady=(10,0), sticky="ew")
        self.entry_1_var = customtkinter.StringVar(value="0.0, 0.0")
        self.entry_1 = customtkinter.CTkEntry(self, textvariable=self.entry_1_var)
        self.entry_1.grid(row=1, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_2_title = customtkinter.CTkLabel(self, text="Right Edge", fg_color="grey50", corner_radius=6)
        self.entry_2_title.grid(row=2, column=0, padx=10, pady=(10,0), sticky="ew")
        self.entry_2_var = customtkinter.StringVar(value="2.8, 0.0")
        self.entry_2 = customtkinter.CTkEntry(self, textvariable=self.entry_2_var)
        self.entry_2.grid(row=2, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_3_title = customtkinter.CTkLabel(self, text="Top-Right Corner", fg_color="grey50", corner_radius=6)
        self.entry_3_title.grid(row=3, column=0, padx=10, pady=(10,0), sticky="ew")
        self.entry_3_var = customtkinter.StringVar(value="2.8, 3.0")
        self.entry_3 = customtkinter.CTkEntry(self, textvariable=self.entry_3_var)
        self.entry_3.grid(row=3, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_4_title = customtkinter.CTkLabel(self, text="Top-Left Corner", fg_color="grey50", corner_radius=6)
        self.entry_4_title.grid(row=4, column=0, padx=10, pady=(10,0), sticky="ew")
        self.entry_4_var = customtkinter.StringVar(value="0.0, 3.0")
        self.entry_4 = customtkinter.CTkEntry(self, textvariable=self.entry_4_var)
        self.entry_4.grid(row=4, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_5_title = customtkinter.CTkLabel(self, text="NA", fg_color="grey50", corner_radius=6)
        self.entry_5_title.grid(row=5, column=0, padx=10, pady=(10,0), sticky="ew")
        self.entry_5 = customtkinter.CTkEntry(self, placeholder_text="NA")
        self.entry_5.grid(row=5, column=1, padx=10, pady=(10, 0), sticky="ew")

        self.entry_6_title = customtkinter.CTkLabel(self, text="NA", fg_color="grey50", corner_radius=6)
        self.entry_6_title.grid(row=6, column=0, padx=10, pady=(10,10), sticky="ew")
        self.entry_6 = customtkinter.CTkEntry(self, placeholder_text="NA")
        self.entry_6.grid(row=6, column=1, padx=10, pady=(10, 10), sticky="ew")
        

        update_point_entries("Rectangle")

    def get(self):
        
        pointCoords = np.array([self.entry_1.get().split(","), self.entry_2.get().split(","), self.entry_3.get().split(","), self.entry_4.get().split(",")])
        float_pointCoords = pointCoords.astype(float)
        pointCoords_dict = []
        
        for i in float_pointCoords:
            pointCoords_dict.append({'x': i[0], 'y': i[1]})

        return pointCoords_dict

class headingLocationFrame(customtkinter.CTkFrame):
    def __init__(self, master, values):
        super().__init__(master)
        self.grid_columnconfigure(0, weight=1)
        self.values = values

        self.textbox_targetHeading_title = customtkinter.CTkLabel(self, text="Target Heading", fg_color="grey50", corner_radius=6)
        self.textbox_targetHeading_title.grid(row=1, column=0, padx=10, pady=(10,0), sticky="ew")
        self.textbox_targetHeading = customtkinter.CTkTextbox(self, width=100, height=10, corner_radius=6)
        self.textbox_targetHeading.delete("0.0", customtkinter.END)
        self.textbox_targetHeading.insert("0.0", text=self.values[0])
        self.textbox_targetHeading.configure(state="disabled")
        self.textbox_targetHeading.grid(row=1, column=1, padx=(0,10), pady=(10,0), sticky="ew")

        self.textbox_actualHeading_title = customtkinter.CTkLabel(self, text="Actual Heading", fg_color="grey50", corner_radius=6)
        self.textbox_actualHeading_title.grid(row=2, column=0, padx=10, pady=(10,0), sticky="ew")
        self.textbox_actualHeading = customtkinter.CTkTextbox(self, width=100, height=10, corner_radius=6)
        self.textbox_actualHeading.delete("0.0", customtkinter.END)
        self.textbox_actualHeading.insert("0.0", text=self.values[1])
        self.textbox_actualHeading.configure(state="disabled")
        self.textbox_actualHeading.grid(row=2, column=1, padx=(0,10), pady=(10,0), sticky="ew")

        self.textbox_targetPoint_title = customtkinter.CTkLabel(self, text="Target Point", fg_color="grey50", corner_radius=6)
        self.textbox_targetPoint_title.grid(row=3, column=0, padx=10, pady=(10,0), sticky="ew")
        self.textbox_targetPoint = customtkinter.CTkTextbox(self, width=100, height=10, corner_radius=6)
        self.textbox_targetPoint.delete("0.0", customtkinter.END)
        self.textbox_targetPoint.insert("0.0", text=self.values[2])
        self.textbox_targetPoint.configure(state="disabled")
        self.textbox_targetPoint.grid(row=3, column=1, padx=(0,10), pady=(10,0), sticky="ew")

        self.textbox_actualPoint_title = customtkinter.CTkLabel(self, text="Actual Point", fg_color="grey50", corner_radius=6)
        self.textbox_actualPoint_title.grid(row=4, column=0, padx=10, pady=(10,10), sticky="ew")
        self.textbox_actualPoint = customtkinter.CTkTextbox(self, width=100, height=10, corner_radius=6)
        self.textbox_actualPoint.delete("0.0", customtkinter.END)
        self.textbox_actualPoint.insert("0.0", text=self.values[3])
        self.textbox_actualPoint.configure(state="disabled")
        self.textbox_actualPoint.grid(row=4, column=1, padx=(0,10), pady=(10,10), sticky="ew")
        

class checkboxFrame(customtkinter.CTkFrame):
    def __init__(self, master, title, values):
        super().__init__(master)
        self.grid_columnconfigure(0, weight=1)
        self.values = values
        self.title = title
        self.checkboxes = []

        self.title = customtkinter.CTkLabel(self, text=self.title, fg_color="grey30", corner_radius=6)
        self.title.grid(row=0, column=0, padx=10, pady=(10,0), sticky="ew")

        for i, value in enumerate(self.values):
            checkbox = customtkinter.CTkCheckBox(self, text=value)
            checkbox.grid(row=i+1, column=0, padx=10, pady=(10, 0), sticky="w")
            self.checkboxes.append(checkbox)

    def get(self):
        checked_checkboxes = []
        for checkbox in self.checkboxes:
            if checkbox.get() == 1:
                checked_checkboxes.append(checkbox.cget('text'))
        return checked_checkboxes

class comboBoxFrame(customtkinter.CTkFrame):
    def __init__(self, master, title, values):
        super().__init__(master)
        self.grid_columnconfigure(0, weight=1)
        self.values = values
        self.title = title

        self.title = customtkinter.CTkLabel(self, text=self.title, fg_color="grey30", corner_radius=6)
        self.title.grid(row=0, column=0, padx=10, pady=(10,0), sticky="ew")

        self.combobox = customtkinter.CTkComboBox(self, values = values)
        self.combobox.grid(row=1, column=0, padx=10, pady=(10, 0), sticky="w")

        self.combobox.set(self.values[0])

    def get(self):
        combobox_value = self.combobox.get()
        return combobox_value

class entryFrame(customtkinter.CTkEntry):
    def __init__(self, master, title, placeholder_texts):
        super().__init__(master)
        self.grid_columnconfigure(0, weight=1)
        self.placeholder_texts = placeholder_texts
        self.title = title
        self.entries = []

        self.title = customtkinter.CTkLabel(self, text=self.title, fg_color="grey30", corner_radius=6)
        self.title.grid(row=0, column=0, padx=10, pady=(10,0), sticky="ew")

        for i, value in enumerate(self.placeholder_texts):
            entry = customtkinter.CTkEntry(self, placeholder_text=value)
            entry.grid(row=i+1, column=0, padx=10, pady=(10, 0), sticky="w")
            self.entries.append(entry)
    def get(self):
        entry_values = []
        for entry in self.entries:
            entry_values.append(entry.get())
        return entry_values 

class App(customtkinter.CTk):

    point_status = 0
    cleaning_status_value = 0

    def __init__(self):
        super().__init__()
    
        

        # Initialize UI and Define Title and Default Window Size
        self.title("Automated Designs")
        self.geometry("768x512")
        self.grid_columnconfigure((0, 1), weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.startcleaning = 0 #0 = false, 1 = true
        self.headingLocationValues = [90, 87, (10, 0), (9.8, 0.4)]
      

        # Create Cleaning Area Parameter Frame
        self.pointEntriesFrame = pointEntriesFrame(self)
        self.pointEntriesFrame.grid(row=0, column=0, padx=(0, 10), pady=10, sticky="nw")

        self.headingLocationFrame = headingLocationFrame(self, values=self.headingLocationValues)
        self.headingLocationFrame.grid(row=0, column=1, padx=(0,10), pady=10, sticky="nw")

        # self.checkbox_frame_1 = checkboxFrame(self, "Values", values=["value 1", "value 2", "value 3"])
        # self.checkbox_frame_1.grid(row=0, column=1, padx=10, pady=(10, 0), sticky="nsw")

        # self.button = customtkinter.CTkButton(self, text="Start Server", command=self.startServer)
        # self.button.grid(row=3, column=0, padx=10, pady=10, sticky="ew", columnspan=1)  
        

        self.button = customtkinter.CTkButton(self, text="Start Cleaning", command=self.export_areaData)
        self.button.grid(row=3, column=1, padx=10, pady=10, sticky="ew", columnspan=2)  

    # def set_points(self):
    #     App.point_status = 1
    #     return App.point_status

    # def set_cleaning_status(self):
    #     if App.cleaning_status_value == 0:
    #         App.cleaning_status_value = 1
    #     else:
    #         App.cleaning_status_value = 0

    def export_areaData(self):
        area_Data = []
        if App.point_status == 1:
            area_Data = self.pointEntriesFrame.get()
        else:
            area_Data= []
        print(area_Data)

        return area_Data

    # def cleaning_status(self):
    #     return App.cleaning_status_value

app = App()
app.mainloop()


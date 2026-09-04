import gradio as gr


# def get_events():
#         pass

def get_index_page(demo,launcher):
    gr.Markdown("## 🤖 ROS2-MCP Portal")

    with gr.Row(elem_classes=["three-pane"]):

            # -------- LEFT PANE --------
            with gr.Column(elem_classes=["left-pane"]):

                #gr.Markdown("## 📡 ROS Interfaces")


                with gr.Group():
                    gr.Markdown("### 🗺️ Live Robot Map")
                    map_img = gr.Image(label="Robot Map",type="numpy", height=275)
                    def update_map():
                        return launcher.map_subscriber.draw_gradio()
                    
                    map_timer = gr.Timer(0.1)
                    map_timer.tick(
                        fn=update_map, #update_pose_for_map,
                        inputs=None,
                        outputs=map_img
                    )

                    map_click_status = gr.Textbox(label="Map Command", lines=1)
                    def on_map_click(evt: gr.EventData):
                        if evt is None or not hasattr(evt, "index") or evt.index is None:
                            return "❌ No click data received. Try clicking the map image."
                        wx, wy = launcher.map_subscriber.click_to_world(evt.index[0], evt.index[1])
                        if wx is None or wy is None:
                            return "⏳ Waiting for map/pose data... (check /map and /utlidar/robot_pose topics)"
                        ok = launcher.nav2_controller.go_to({"x": wx, "y": wy, "yaw": launcher.map_subscriber.yaw})
                        if not ok:
                            return "❌ Nav2 navigate_to_pose server unavailable. Check `ros2 action list`."
                        return f"🚀 Navigate to ({wx:.2f}, {wy:.2f})"

                    map_img.select(
                        fn=on_map_click,
                        inputs=None,
                        outputs=map_click_status
                    )


                gr.Markdown("### ⚙️ Sports Mode Actions")

                dropdown = gr.Dropdown(label="Select Action", choices=launcher.action_subscriber.available_actions)
                btn = gr.Button("Run Action")
                output = gr.Textbox(label="Status")

                btn.click(launcher.run_action, inputs=dropdown, outputs=output)

                with gr.Group():
                    motor_val = gr.Textbox(label=" ⚙️🌡️ Motor Temp", lines=2)

                def update_motor_wrapper():

                    motor = launcher.bm_subscriber.get_motor_data()

                    return motor
                
                timer = gr.Timer(0.5)
                timer.tick(
                    fn=update_motor_wrapper,
                    inputs=None,
                    outputs=[

                        motor_val,
                    ]
                )

            # -------- CENTER PANE (SMALL CAMERAS + MAIN CAMERA) --------
            with gr.Column(elem_classes=["center-pane"]):


                gr.Markdown("### 🔹 Additional Views")

                cam_status = gr.Textbox(label="📷 Camera Status", lines=1, interactive=False)

                def update_cam_status():
                    return launcher.camera_status()

                demo.load(update_cam_status, None, cam_status)
                cam_timer = gr.Timer(2.0)
                cam_timer.tick(fn=update_cam_status, inputs=None, outputs=cam_status)

                # --- THREE SMALL CAMERAS ---
                with gr.Row(elem_id="small-cam-row"):
                    depth_cam = gr.Image(label="RealSense_Depth", type="numpy", height=150)
                    seg_cam   = gr.Image(label="RealSense_Color", type="numpy", height=150)
                    yolo_cam  = gr.Image(label="YOLO", type="numpy", height=150)

                def depth_feed():
                    for frame, _ in launcher.live_cam_feed("rs_depth_image"):
                        yield frame

                def seg_feed():
                    for frame, _ in launcher.live_cam_feed("rs_color_image"):
                        yield frame

                def yolo_feed():
                    for frame, _ in launcher.live_cam_feed("yolo_detection"):
                        yield frame



                demo.load(depth_feed, None, depth_cam)
                demo.load(seg_feed, None, seg_cam)
                demo.load(yolo_feed, None, yolo_cam)

                # --- MAIN CAMERA ---
                gr.Markdown("### 📸 Main Camera")
                live_image = gr.Image(type="numpy", height=420)
                live_text = gr.Textbox(label="Description", lines=3)

                def live_feed_wrapper():
                    yield from launcher.live_cam_feed("color_image")

                demo.load(
                    fn=live_feed_wrapper,
                    inputs=None,
                    outputs=[live_image, live_text]
                )


            # -------- RIGHT PANE --------
            with gr.Column(elem_classes=["right-pane"]):


                with gr.Row(elem_classes=["control-grid"]):
                    rotate_left_btn = gr.Button("⟲", elem_id="rotate_left")
                    forward_btn = gr.Button("↑", elem_id="forward")
                    rotate_right_btn = gr.Button("⟳", elem_id="rotate_right")

                with gr.Row(elem_classes=["control-grid"]):
                    left_btn = gr.Button("←", elem_id="left")
                    stop_btn = gr.Button("■", elem_id="stop")
                    right_btn = gr.Button("→", elem_id="right")

                with gr.Row(elem_classes=["control-grid"]):
                    gr.Button("", elem_id="empty")
                    backward_btn = gr.Button("↓", elem_id="backward")
                    gr.Button("", elem_id="empty")

                # stop_btn.click(launcher.controller.stop_robot, [], [feedback_box])

                forward_btn.click(launcher.wirelesscontroller.move_forward,inputs=[],outputs=[])#event_box
                backward_btn.click(launcher.wirelesscontroller.move_backward,inputs=[],outputs=[])
                left_btn.click(launcher.wirelesscontroller.move_left,inputs=[],outputs=[])
                right_btn.click(launcher.wirelesscontroller.move_right,inputs=[],outputs=[])
                rotate_left_btn.click(launcher.wirelesscontroller.rotate_left,inputs=[],outputs=[])
                rotate_right_btn.click(launcher.wirelesscontroller.rotate_right,inputs=[],outputs=[])
                stop_btn.click(launcher.wirelesscontroller.stop_robot,inputs=[],outputs=[])



                with gr.Row():

                    
                    with gr.Row():

                        # --- Position Box ---
                        with gr.Group():
                            position_val = gr.Textbox(label=" 📍 Position", lines=1)

                        # --- Orientation Box ---
                        with gr.Group():
                            orientation_val = gr.Textbox(label=" 🧭 Orientation (radians)", lines=1)

                        # --- Velocity Box ---
                        with gr.Group():
                            velocity_val = gr.Textbox(label=" ⚡ Velocity", lines=1)
                        
                        with gr.Group():
                            battery_val = gr.Textbox(label=" 🔋 Battery", lines=2)
                        
                        with gr.Group():
                            system_val = gr.Textbox(label="⚙️ System", lines=1)


                        def update_all_wrapper():
                            pos, _, vel = launcher.map_subscriber.get_live_data()
                            orient =launcher.bm_subscriber.get_orientation_data()
                            battery = launcher.bm_subscriber.get_battery_data()
                            system = launcher.bm_subscriber.get_system_state_data()


                            return pos, orient, vel, battery, system
                        
                        timer = gr.Timer(0.5)
                        timer.tick(
                            fn=update_all_wrapper,
                            inputs=None,
                            outputs=[
                                position_val,
                                orientation_val,
                                velocity_val,
                                battery_val,
                                system_val
                            ]
                        )

                    # )
    # event_timer = gr.Timer(1.0)
    # event_timer.tick(
    #     fn=get_events,
    #     inputs=None,
    #     outputs=None
    # )
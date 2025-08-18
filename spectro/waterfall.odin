package spectro
import rl "vendor:raylib"

draw_waterfall :: proc(app: ^App, bounds: rl.Rectangle) {
	using rl

	if app.settings.waterfall_show_panel {
		GuiPanel(bounds, "Waterfall")	
	}
	

	BeginShaderMode(app.scroll_shader)

	// Bind textures while shader is active (matches C version approach)
	write_head_float := f32(app.write_head)
	SetShaderValue(
		app.scroll_shader,
		app.write_head_loc,
		&write_head_float,
		ShaderUniformDataType.FLOAT,
	)
	SetShaderValueTexture(app.scroll_shader, app.ring_tex_loc, app.ring_texture)
	SetShaderValueTexture(app.scroll_shader, app.lut_tex_loc, app.lut_texture)


	// Draw waterfall starting at y=300 to leave room for GUI controls
	DrawTexturePro(
		app.ring_texture,
		Rectangle{0, 0, f32(app.ring_texture.width), f32(app.ring_texture.height)},
		Rectangle {
			bounds.x + 1,
			bounds.y + 25,
			bounds.width - 2,
			bounds.height - 26,
		},
		Vector2{0, 0},
		0.0,
		WHITE,
	)
	EndShaderMode()
}
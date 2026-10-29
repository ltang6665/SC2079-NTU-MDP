package com.example.android_app.arena

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.PointF
import android.graphics.RectF
import android.graphics.Typeface
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import androidx.core.content.ContextCompat
import android.graphics.Bitmap
import android.graphics.Matrix
import androidx.core.graphics.drawable.toBitmap
import com.example.android_app.R

/**
 * Custom View rendering the 20x20 exploration arena.
 * Target ID display (set via [setTargetId])
 * Robot pose update (set via [setRobot])
 *
 * Callbacks:
 *   [onObstaclePlaced]     — new obstacle put on arena
 *   [onObstacleMoved]      — existing obstacle dragged
 *   [onObstacleDropped]    — shift obstacle position
 *   [onObstacleRemoved]    — dragged outside arena
 *   [onObstacleLongPress]  — hosting activity opens the face-picker dialog
 */
class ArenaView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0,
) : View(context, attrs, defStyle) {

    companion object {
        const val COLS = 20
        const val ROWS = 20

        // Start zone is a 4x4 in the bottom-left
        const val START_ZONE_SIZE = 4
        private const val LONG_PRESS_MS = 400L
        private const val DRAG_SLOP_PX = 12f
    }

    var onObstaclePlaced: ((Obstacle) -> Unit)? = null

    // replace all move instructions with drop so as to prevent continuous transmission even when in the midst of moving
    // var onObstacleMoved: ((Obstacle) -> Unit)? = null
    var onObstacleDropped: ((Obstacle) -> Unit)? = null

    var onObstacleRemoved: ((Obstacle) -> Unit)? = null
    var onObstacleLongPress: ((Obstacle) -> Unit)? = null

    private val obstacles = mutableListOf<Obstacle>()
    private var nextObstacleId = 1
    val robot: Robot = Robot(cellX = 1, cellY = 1, facing = Facing.NORTH)
    private var robotBitmap: Bitmap? = null

    // ── Paints ──
    private val paintBg = Paint().apply { style = Paint.Style.FILL }
    private val paintGridMinor = Paint().apply {
        style = Paint.Style.STROKE
        strokeWidth = 1f
    }
    private val paintGridMajor = Paint().apply {
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }
    private val paintStartZone = Paint().apply { style = Paint.Style.FILL }
    private val paintObstacle = Paint().apply { style = Paint.Style.FILL }
    private val paintObstacleFace = Paint().apply { style = Paint.Style.FILL }
    private val paintObstacleTarget = Paint().apply { style = Paint.Style.FILL }
    private val paintObstacleText = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
        textAlign = Paint.Align.CENTER
    }
    private val paintAxisText = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
        textAlign = Paint.Align.CENTER
    }
    private val paintRobotBody = Paint().apply { style = Paint.Style.FILL }
    private val paintRobotFacing = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val paintRobotHeadingBg = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val paintRobotHeadingText = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        textAlign = Paint.Align.CENTER
        typeface = Typeface.DEFAULT_BOLD
    }

    private var cellSize: Float = 0f
    private var boardLeft: Float = 0f
    private var boardTop: Float = 0f

    private var draggingObstacle: Obstacle? = null
    private var pressDownX = 0f
    private var pressDownY = 0f
    private var pressDownAt = 0L
    private var pressedObstacle: Obstacle? = null
    private var didDrag = false
    private val longPressRunnable = Runnable {
        val obs = pressedObstacle ?: return@Runnable
        if (!didDrag) {
            onObstacleLongPress?.invoke(obs)
            pressedObstacle = null
        }
    }

    init {
        paintBg.color = ContextCompat.getColor(context, R.color.grid_background)
        paintGridMinor.color = ContextCompat.getColor(context, R.color.grid_line)
        paintGridMajor.color = ContextCompat.getColor(context, R.color.grid_line_major)
        paintStartZone.color = ContextCompat.getColor(context, R.color.start_zone)
        paintObstacle.color = ContextCompat.getColor(context, R.color.obstacle)
        paintObstacleFace.color = ContextCompat.getColor(context, R.color.obstacle_face)
        paintObstacleTarget.color = ContextCompat.getColor(context, R.color.obstacle_target)
        paintObstacleText.color = ContextCompat.getColor(context, R.color.obstacle_text)
        paintRobotBody.color = ContextCompat.getColor(context, R.color.robot_body)
        paintRobotFacing.color = ContextCompat.getColor(context, R.color.robot_facing)
        paintAxisText.color = ContextCompat.getColor(context, R.color.arena_axis_text)
        paintRobotHeadingBg.color = ContextCompat.getColor(context, R.color.grid_background)
        paintRobotHeadingText.color = ContextCompat.getColor(context, R.color.obstacle_text)
        val drawable = ContextCompat.getDrawable(context, R.drawable.robot_car)
        robotBitmap = drawable?.toBitmap()
    }

    // ── Public API ──

    /** Reset arena to a clean state (called by Reset button). */
    fun reset() {
        obstacles.clear()
        nextObstacleId = 1
        robot.cellX = 1
        robot.cellY = 1
        robot.facing = Facing.NORTH
        invalidate()
    }

    fun obstacles(): List<Obstacle> = obstacles.toList()

    fun findObstacle(id: Int): Obstacle? = obstacles.firstOrNull { it.id == id }

    /** C.9  RPi told us obstacle N was recognised as target M. */
    fun setTargetId(obstacleId: Int, targetId: Int) {
        val obs = findObstacle(obstacleId) ?: return
        obs.targetId = targetId
        invalidate()
    }

    /** C.10 RPi told us robot moved. */
    fun setRobot(x: Int, y: Int, facing: Facing) {
        robot.cellX = x.coerceIn(0, COLS - 1)
        robot.cellY = y.coerceIn(0, ROWS - 1)
        robot.facing = facing
        invalidate()
    }

    /** C.7  set the target face after picker dialog closed. */
    fun setObstacleFace(obstacleId: Int, face: Facing?) {
        val obs = findObstacle(obstacleId) ?: return
        obs.face = face
        invalidate()
    }

    // ── Layout math ──

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        recomputeGeometry(w, h)
    }

    private fun recomputeGeometry(w: Int, h: Int) {
        val padding = 40f
        val boardMax = minOf(w - padding, h - padding)
        cellSize = boardMax / COLS
        boardLeft = (w - cellSize * COLS) / 2f + (padding / 4f)
        boardTop = (h - cellSize * ROWS) / 2f - (padding / 4f)

        paintObstacleText.textSize = cellSize * 0.55f
        paintAxisText.textSize = minOf(cellSize * 0.70f, resources.getDimension(R.dimen.arena_axis_text_size))
    }

    private fun cellToRect(cellX: Int, cellY: Int): RectF {
        // cellY grows upward, so we flip when converting to screen coords.
        val left = boardLeft + cellX * cellSize
        val top = boardTop + (ROWS - 1 - cellY) * cellSize
        return RectF(left, top, left + cellSize, top + cellSize)
    }

    private fun screenToCell(x: Float, y: Float): Pair<Int, Int>? {
        val relX = x - boardLeft
        val relY = y - boardTop
        val cx = (relX / cellSize).toInt()
        val cyFromTop = (relY / cellSize).toInt()
        val cy = ROWS - 1 - cyFromTop
        if (cx !in 0 until COLS || cy !in 0 until ROWS) return null
        return cx to cy
    }

    // ── Rendering ──

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        drawBackground(canvas)
        drawStartZone(canvas)
        drawGrid(canvas)
        drawAxisLabels(canvas)
        drawObstacles(canvas)
        drawRobot(canvas)
    }

    private fun drawBackground(canvas: Canvas) {
        canvas.drawRect(
            boardLeft, boardTop,
            boardLeft + cellSize * COLS,
            boardTop + cellSize * ROWS,
            paintBg,
        )
    }

    private fun drawGrid(canvas: Canvas) {
        for (i in 0..COLS) {
            val x = boardLeft + i * cellSize
            val paint = if (i % 5 == 0) paintGridMajor else paintGridMinor
            canvas.drawLine(x, boardTop, x, boardTop + cellSize * ROWS, paint)
        }
        for (i in 0..ROWS) {
            val y = boardTop + i * cellSize
            val paint = if (i % 5 == 0) paintGridMajor else paintGridMinor
            canvas.drawLine(boardLeft, y, boardLeft + cellSize * COLS, y, paint)
        }
    }

    private fun drawAxisLabels(canvas: Canvas) {
        val yOffset = cellSize * 0.25f

        for (i in 0 until COLS) {
            val cellRect = cellToRect(i, 0)

            // X-axis numbers (0 to 19 along the bottom)
            canvas.drawText(
                i.toString(),
                cellRect.centerX(),
                boardTop + cellSize * ROWS + yOffset + paintAxisText.textSize,
                paintAxisText
            )

            // Y-axis numbers (0 to 19 along the left)
            val yCellRect = cellToRect(0, i)
            canvas.drawText(
                i.toString(),
                boardLeft - yOffset - (paintAxisText.textSize / 2f),
                yCellRect.centerY() + (paintAxisText.textSize / 3f),
                paintAxisText
            )
        }
    }

    private fun drawStartZone(canvas: Canvas) {
        // 4x4 bottom-left
        val rect = RectF(
            boardLeft,
            boardTop + cellSize * (ROWS - START_ZONE_SIZE),
            boardLeft + cellSize * START_ZONE_SIZE,
            boardTop + cellSize * ROWS,
        )
        canvas.drawRect(rect, paintStartZone)
    }

    private fun drawObstacles(canvas: Canvas) {
        for (obs in obstacles) {
            val rect = cellToRect(obs.cellX, obs.cellY)
            val bodyPaint = if (obs.targetId != null) paintObstacleTarget else paintObstacle
            canvas.drawRect(rect, bodyPaint)
            drawObstacleFace(canvas, obs, rect)
            drawObstacleLabel(canvas, obs, rect)
        }
    }

    private fun drawObstacleFace(canvas: Canvas, obs: Obstacle, rect: RectF) {
        val face = obs.face ?: return
        val thickness = cellSize * 0.18f
        val faceRect = when (face) {
            Facing.NORTH -> RectF(rect.left, rect.top, rect.right, rect.top + thickness)
            Facing.SOUTH -> RectF(rect.left, rect.bottom - thickness, rect.right, rect.bottom)
            Facing.EAST -> RectF(rect.right - thickness, rect.top, rect.right, rect.bottom)
            Facing.WEST -> RectF(rect.left, rect.top, rect.left + thickness, rect.bottom)
        }
        canvas.drawRect(faceRect, paintObstacleFace)
    }

    private fun drawObstacleLabel(canvas: Canvas, obs: Obstacle, rect: RectF) {
        val label = obs.targetId?.toString() ?: obs.id.toString()
        // Small setup IDs become prominent target IDs when recognition succeeds (C.5 / C.9).
        paintObstacleText.textSize = cellSize * if (obs.targetId != null) 0.66f else 0.48f
        paintObstacleText.typeface = if (obs.targetId != null) Typeface.DEFAULT_BOLD else Typeface.DEFAULT
        val maxWidth = cellSize * 0.70f
        val measuredWidth = paintObstacleText.measureText(label)
        if (measuredWidth > maxWidth) {
            paintObstacleText.textSize *= maxWidth / measuredWidth
        }
        // Centre the label in the space remaining beside the target-face stripe.
        val faceOffset = cellSize * 0.09f
        val cx = rect.centerX() + when (obs.face) {
            Facing.WEST -> faceOffset
            Facing.EAST -> -faceOffset
            else -> 0f
        }
        val labelCenterY = rect.centerY() + when (obs.face) {
            Facing.NORTH -> faceOffset
            Facing.SOUTH -> -faceOffset
            else -> 0f
        }
        val cy = labelCenterY - (paintObstacleText.ascent() + paintObstacleText.descent()) / 2f
        canvas.drawText(label, cx, cy, paintObstacleText)
    }

    private fun drawRobot(canvas: Canvas) {
        // Robot occupies 3x3 centred on (robot.cellX, robot.cellY)
        val leftCell = robot.cellX - 1
        val bottomCell = robot.cellY - 1
        val topLeft = cellToRect(leftCell, bottomCell + 2)
        val bottomRight = cellToRect(leftCell + 2, bottomCell)
        val body = RectF(topLeft.left, topLeft.top, bottomRight.right, bottomRight.bottom)
        canvas.drawRect(body, paintRobotBody)

        // Facing arrow — a triangle pointing in [robot.facing]
        val cx = body.centerX()
        val cy = body.centerY()
        val r = body.width() * 0.28f
        val tip: PointF
        val baseA: PointF
        val baseB: PointF
        when (robot.facing) {
            Facing.NORTH -> {
                tip = PointF(cx, cy - r)
                baseA = PointF(cx - r * 0.6f, cy + r * 0.4f)
                baseB = PointF(cx + r * 0.6f, cy + r * 0.4f)
            }

            Facing.SOUTH -> {
                tip = PointF(cx, cy + r)
                baseA = PointF(cx - r * 0.6f, cy - r * 0.4f)
                baseB = PointF(cx + r * 0.6f, cy - r * 0.4f)
            }

            Facing.EAST -> {
                tip = PointF(cx + r, cy)
                baseA = PointF(cx - r * 0.4f, cy - r * 0.6f)
                baseB = PointF(cx - r * 0.4f, cy + r * 0.6f)
            }

            Facing.WEST -> {
                tip = PointF(cx - r, cy)
                baseA = PointF(cx + r * 0.4f, cy - r * 0.6f)
                baseB = PointF(cx + r * 0.4f, cy + r * 0.6f)
            }
        }
        val path = android.graphics.Path().apply {
            moveTo(tip.x, tip.y)
            lineTo(baseA.x, baseA.y)
            lineTo(baseB.x, baseB.y)
            close()
        }
        canvas.drawPath(path, paintRobotFacing)

        // A compass badge reinforces the arrow without extending outside the 3x3 footprint.
        val headingOffset = cellSize * 0.95f
        val headingX = cx + when (robot.facing) {
            Facing.EAST -> -headingOffset
            Facing.WEST -> headingOffset
            else -> 0f
        }
        val headingY = cy + when (robot.facing) {
            Facing.NORTH -> headingOffset
            Facing.SOUTH -> -headingOffset
            else -> 0f
        }
        canvas.drawCircle(headingX, headingY, cellSize * 0.32f, paintRobotHeadingBg)
        paintRobotHeadingText.textSize = cellSize * 0.46f
        val headingBaseline = headingY - (paintRobotHeadingText.ascent() + paintRobotHeadingText.descent()) / 2f
        canvas.drawText(robot.facing.name.take(1), headingX, headingBaseline, paintRobotHeadingText)

        /* val bmp = robotBitmap
        if (bmp != null) {
            val rotationDegrees = when (robot.facing) {
                Facing.NORTH -> 0f
                Facing.EAST  -> 90f
                Facing.SOUTH -> 180f
                Facing.WEST  -> 270f
            }

            val matrix = Matrix().apply {
                // 1. Rotate around the unscaled bitmap's center point
                postRotate(rotationDegrees, bmp.width / 2f, bmp.height / 2f)

                // 2. Scale the rotated bitmap to match the 3x3 grid size
                postScale(body.width() / bmp.width, body.height() / bmp.height)

                // 3. Move the bitmap into position on the canvas
                postTranslate(body.left, body.top)
            }

            canvas.drawBitmap(bmp, matrix, null)
        } else {
            canvas.drawRect(body, paintRobotBody)
        }*/
    }

    // ── Touch ──

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                pressDownX = event.x
                pressDownY = event.y
                didDrag = false
                pressedObstacle = obstacleAt(event.x, event.y)
                draggingObstacle = pressedObstacle
                if (pressedObstacle != null) {
                    postDelayed(longPressRunnable, LONG_PRESS_MS)
                }
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                val dx = event.x - pressDownX
                val dy = event.y - pressDownY
                if (!didDrag && (dx * dx + dy * dy) > DRAG_SLOP_PX * DRAG_SLOP_PX) {
                    didDrag = true
                    removeCallbacks(longPressRunnable)
                }
                val dragging = draggingObstacle ?: return true
                val cell = screenToCell(event.x, event.y)

                // Update UI locally (no network transmission)
                if (cell != null && (dragging.cellX != cell.first || dragging.cellY != cell.second)) {
                    dragging.cellX = cell.first
                    dragging.cellY = cell.second
                    invalidate()
                }
                return true
            }

            MotionEvent.ACTION_UP -> {
                removeCallbacks(longPressRunnable)
                val dragging = draggingObstacle
                val cell = screenToCell(event.x, event.y)

                if (dragging != null) {
                    if (cell == null) {
                        // Dragged outside -> delete
                        obstacles.remove(dragging)
                        onObstacleRemoved?.invoke(dragging)
                    } else if (didDrag) {
                        // Drop finished -> transmit final position to RPi
                        dragging.cellX = cell.first
                        dragging.cellY = cell.second
                        onObstacleDropped?.invoke(dragging)
                    }
                } else if (cell != null && !didDrag) {
                    // Tap on empty cell -> place new obstacle
                    val obs = Obstacle(nextObstacleId++, cell.first, cell.second)
                    obstacles.add(obs)
                    onObstaclePlaced?.invoke(obs)
                }

                invalidate()
                draggingObstacle = null
                pressedObstacle = null
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                removeCallbacks(longPressRunnable)
                draggingObstacle = null
                pressedObstacle = null
                didDrag = false
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    private fun obstacleAt(x: Float, y: Float): Obstacle? {
        val cell = screenToCell(x, y) ?: return null
        return obstacles.firstOrNull { it.cellX == cell.first && it.cellY == cell.second }
    }
}

-- Add the pathfinder module to the package path.
package.path = "../?.lua;" .. package.path
package.path = "../?/pathfinder.lua;" .. package.path

local pathfinder = require("pathfinder")
local Player = require("player")

local player
local mainPolygon
local obstacles
local debugMode = true
local showNavMesh = false
local showPortals = false
local showCorners = false
local navVertices, navEdges
local lastPath = nil
local lastRawPath = nil -- Store the raw A* path before smoothing
local lastPortals = nil
local lastCorners = nil
local gridSize = 30 -- Grid size for navigation mesh

-- Pathfinding options
local pathOptions = {
	gridSize = 30,
	portalWidth = 15,
	cornerRadius = 8,
	useFunnel = true,
}

function love.load()
	-- Set up the window
	love.window.setTitle("Enhanced Pathfinding Visualization with Funnel Algorithm")
	love.window.setMode(1000, 700)

	-- Create the player
	player = Player:new()

	-- Define the main polygon (room)
	mainPolygon = {
		{ x = 50, y = 50 },
		{ x = 950, y = 50 },
		{ x = 950, y = 650 },
		{ x = 50, y = 650 },
	}

	-- Define obstacles (furniture, etc.)
	obstacles = {
		-- Obstacle 1 (table)
		{
			{ x = 200, y = 200 },
			{ x = 300, y = 200 },
			{ x = 300, y = 300 },
			{ x = 200, y = 300 },
		},
		-- Obstacle 2 (L-shaped couch)
		{
			{ x = 500, y = 150 },
			{ x = 650, y = 150 },
			{ x = 650, y = 200 },
			{ x = 550, y = 200 },
			{ x = 550, y = 300 },
			{ x = 500, y = 300 },
		},
		-- Obstacle 3 (circular table approximated as polygon)
		{
			{ x = 400, y = 400 },
			{ x = 420, y = 380 },
			{ x = 450, y = 370 },
			{ x = 480, y = 380 },
			{ x = 500, y = 400 },
			{ x = 480, y = 420 },
			{ x = 450, y = 430 },
			{ x = 420, y = 420 },
		},
		-- Obstacle 4 (narrow corridor obstacle)
		{
			{ x = 700, y = 400 },
			{ x = 750, y = 400 },
			{ x = 750, y = 600 },
			{ x = 700, y = 600 },
		},
		-- Obstacle 5 (complex shape)
		{
			{ x = 100, y = 450 },
			{ x = 150, y = 450 },
			{ x = 150, y = 500 },
			{ x = 200, y = 500 },
			{ x = 200, y = 550 },
			{ x = 100, y = 550 },
		},
	}

	-- Generate the navigation mesh
	navVertices, navEdges = pathfinder.getNavigationMesh(mainPolygon, obstacles, gridSize)
end

function love.update(dt)
	player:update(dt)
end

-- Helper function to create simple portals for visualization
local function createSimplePortals(path)
	if not path or #path < 3 then
		return {}
	end

	local portals = {}
	local portalWidth = pathOptions.portalWidth

	for i = 2, #path - 1 do
		local prev = path[i - 1]
		local current = path[i]
		local next = path[i + 1]

		-- Calculate direction vectors
		local dir1 = { x = current.x - prev.x, y = current.y - prev.y }
		local dir2 = { x = next.x - current.x, y = next.y - current.y }

		-- Normalize direction vectors
		local len1 = math.sqrt(dir1.x * dir1.x + dir1.y * dir1.y)
		local len2 = math.sqrt(dir2.x * dir2.x + dir2.y * dir2.y)

		if len1 > 0 and len2 > 0 then
			dir1.x, dir1.y = dir1.x / len1, dir1.y / len1
			dir2.x, dir2.y = dir2.x / len2, dir2.y / len2

			-- Calculate average direction
			local avgDir = {
				x = (dir1.x + dir2.x) / 2,
				y = (dir1.y + dir2.y) / 2,
			}

			-- Normalize average direction
			local avgLen = math.sqrt(avgDir.x * avgDir.x + avgDir.y * avgDir.y)
			if avgLen > 0 then
				avgDir.x, avgDir.y = avgDir.x / avgLen, avgDir.y / avgLen

				-- Create perpendicular vector for portal
				local perpDir = { x = -avgDir.y, y = avgDir.x }

				-- Create portal points
				local portalLeft = {
					x = current.x + perpDir.x * portalWidth / 2,
					y = current.y + perpDir.y * portalWidth / 2,
				}
				local portalRight = {
					x = current.x - perpDir.x * portalWidth / 2,
					y = current.y - perpDir.y * portalWidth / 2,
				}

				table.insert(portals, {
					left = portalLeft,
					right = portalRight,
					center = current,
				})
			end
		end
	end

	return portals
end

-- Helper function to detect corners for visualization
local function detectSimpleCorners(path)
	if not path or #path < 3 then
		return {}
	end

	local corners = {}
	local angleThreshold = math.pi / 6 -- 30 degrees

	for i = 2, #path - 1 do
		local prev = path[i - 1]
		local current = path[i]
		local next = path[i + 1]

		-- Calculate vectors
		local v1 = { x = current.x - prev.x, y = current.y - prev.y }
		local v2 = { x = next.x - current.x, y = next.y - current.y }

		-- Calculate angle between vectors
		local dot = v1.x * v2.x + v1.y * v2.y
		local cross = v1.x * v2.y - v1.y * v2.x
		local angle = math.atan2(cross, dot)

		if math.abs(angle) > angleThreshold then
			table.insert(corners, {
				index = i,
				point = current,
				angle = angle,
				isSharp = math.abs(angle) > math.pi / 3, -- 60 degrees
			})
		end
	end

	return corners
end

function love.draw()
	-- Draw the main polygon (room) in blue
	love.graphics.setColor(0.2, 0.3, 0.8, 0.2)
	drawPolygon(mainPolygon)
	love.graphics.setColor(0.2, 0.3, 0.8, 0.8)
	drawPolygonOutline(mainPolygon)

	-- Draw the obstacles in red
	love.graphics.setColor(0.8, 0.2, 0.2, 0.6)
	for _, obstacle in ipairs(obstacles) do
		drawPolygon(obstacle)
	end
	love.graphics.setColor(0.8, 0.2, 0.2, 1.0)
	for _, obstacle in ipairs(obstacles) do
		drawPolygonOutline(obstacle)
	end

	-- Draw the navigation mesh if enabled
	if showNavMesh then
		-- Draw edges
		love.graphics.setColor(0.5, 0.5, 0.5, 0.3)
		love.graphics.setLineWidth(1)
		for _, edge in ipairs(navEdges) do
			love.graphics.line(edge[1].x, edge[1].y, edge[2].x, edge[2].y)
		end

		-- Draw vertices
		love.graphics.setColor(0.8, 0.6, 0.2, 0.7)
		for _, vertex in ipairs(navVertices) do
			love.graphics.circle("fill", vertex.x, vertex.y, 2)
		end
	end

	-- Draw raw path (before smoothing) if available
	if lastRawPath and #lastRawPath > 1 then
		love.graphics.setColor(0.8, 0.8, 0.2, 0.5)
		love.graphics.setLineWidth(2)
		for i = 1, #lastRawPath - 1 do
			love.graphics.line(lastRawPath[i].x, lastRawPath[i].y, lastRawPath[i + 1].x, lastRawPath[i + 1].y)
		end

		-- Draw raw waypoints
		love.graphics.setColor(0.8, 0.8, 0.2, 0.8)
		for _, waypoint in ipairs(lastRawPath) do
			love.graphics.circle("fill", waypoint.x, waypoint.y, 2)
		end
	end

	-- Draw portals if enabled and available
	if showPortals and lastPortals and #lastPortals > 0 then
		love.graphics.setColor(0.2, 0.8, 0.2, 0.6)
		love.graphics.setLineWidth(3)
		for _, portal in ipairs(lastPortals) do
			love.graphics.line(portal.left.x, portal.left.y, portal.right.x, portal.right.y)
			-- Draw portal center
			love.graphics.setColor(0.2, 0.8, 0.2, 1.0)
			love.graphics.circle("fill", portal.center.x, portal.center.y, 3)
			love.graphics.setColor(0.2, 0.8, 0.2, 0.6)
		end
	end

	-- Draw corners if enabled and available
	if showCorners and lastCorners and #lastCorners > 0 then
		love.graphics.setColor(1.0, 0.5, 0.0, 0.8)
		for _, corner in ipairs(lastCorners) do
			local radius = corner.isSharp and 6 or 4
			love.graphics.circle("fill", corner.point.x, corner.point.y, radius)

			-- Draw angle indicator
			love.graphics.setColor(1.0, 0.5, 0.0, 0.6)
			love.graphics.circle("line", corner.point.x, corner.point.y, radius + 3)
			love.graphics.setColor(1.0, 0.5, 0.0, 0.8)
		end
	end

	-- Draw the current path (smoothed)
	if lastPath and #lastPath > 1 then
		love.graphics.setColor(0.2, 0.8, 0.2, 0.8)
		love.graphics.setLineWidth(4)
		for i = 1, #lastPath - 1 do
			love.graphics.line(lastPath[i].x, lastPath[i].y, lastPath[i + 1].x, lastPath[i + 1].y)
		end

		-- Draw waypoints
		love.graphics.setColor(0.2, 0.8, 0.2, 1.0)
		for i, waypoint in ipairs(lastPath) do
			local radius = (i == 1 or i == #lastPath) and 5 or 3
			love.graphics.circle("fill", waypoint.x, waypoint.y, radius)
		end
	end

	-- Draw the player
	player:draw()

	-- Draw debug info
	if debugMode then
		love.graphics.setColor(1, 1, 1, 1)
		local y = 10
		local lineHeight = 20

		love.graphics.print("=== Enhanced Pathfinding with Funnel Algorithm ===", 10, y)
		y = y + lineHeight

		love.graphics.print("Click anywhere to set a path for the player", 10, y)
		y = y + lineHeight

		love.graphics.print("Controls:", 10, y)
		y = y + lineHeight

		love.graphics.print("  N - Toggle navigation mesh", 10, y)
		y = y + lineHeight

		love.graphics.print("  P - Toggle portals visualization", 10, y)
		y = y + lineHeight

		love.graphics.print("  C - Toggle corners visualization", 10, y)
		y = y + lineHeight

		love.graphics.print("  F - Toggle funnel algorithm", 10, y)
		y = y + lineHeight

		love.graphics.print("  D - Toggle debug info", 10, y)
		y = y + lineHeight

		love.graphics.print("  +/- - Change grid size", 10, y)
		y = y + lineHeight

		love.graphics.print("  1/2 - Adjust portal width", 10, y)
		y = y + lineHeight

		love.graphics.print("  3/4 - Adjust corner radius", 10, y)
		y = y + lineHeight

		love.graphics.print("  R - Reset settings", 10, y)
		y = y + lineHeight + 10

		love.graphics.print("Settings:", 10, y)
		y = y + lineHeight

		love.graphics.print("  Grid size: " .. pathOptions.gridSize, 10, y)
		y = y + lineHeight

		love.graphics.print("  Portal width: " .. pathOptions.portalWidth, 10, y)
		y = y + lineHeight

		love.graphics.print("  Corner radius: " .. pathOptions.cornerRadius, 10, y)
		y = y + lineHeight

		love.graphics.print("  Funnel algorithm: " .. (pathOptions.useFunnel and "ON" or "OFF"), 10, y)
		y = y + lineHeight + 10

		love.graphics.print("Status:", 10, y)
		y = y + lineHeight

		love.graphics.print("  Player: (" .. math.floor(player.x) .. ", " .. math.floor(player.y) .. ")", 10, y)
		y = y + lineHeight

		love.graphics.print("  Nav mesh nodes: " .. #navVertices, 10, y)
		y = y + lineHeight

		if lastPath then
			love.graphics.print("  Smoothed path waypoints: " .. #lastPath, 10, y)
			y = y + lineHeight

			if lastRawPath then
				love.graphics.print("  Raw path waypoints: " .. #lastRawPath, 10, y)
				y = y + lineHeight
			end

			love.graphics.print("  Current waypoint: " .. (player.currentPathIndex or "N/A"), 10, y)
			y = y + lineHeight
		else
			love.graphics.print("  No path set", 10, y)
			y = y + lineHeight
		end

		if lastPortals and #lastPortals > 0 then
			love.graphics.print("  Portals: " .. #lastPortals, 10, y)
			y = y + lineHeight
		end

		if lastCorners and #lastCorners > 0 then
			love.graphics.print("  Corners: " .. #lastCorners, 10, y)
			y = y + lineHeight
		end
	end

	-- Draw legend
	if debugMode then
		local legendX = love.graphics.getWidth() - 220
		local legendY = 10
		local lineHeight = 20

		love.graphics.setColor(1, 1, 1, 0.9)
		love.graphics.rectangle("fill", legendX - 10, legendY - 5, 210, 160)

		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Legend:", legendX, legendY)
		legendY = legendY + lineHeight

		-- Navigation mesh
		love.graphics.setColor(0.8, 0.6, 0.2, 1)
		love.graphics.circle("fill", legendX + 5, legendY + 8, 3)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Nav mesh", legendX + 15, legendY)
		legendY = legendY + lineHeight

		-- Raw path
		love.graphics.setColor(0.8, 0.8, 0.2, 1)
		love.graphics.rectangle("fill", legendX + 2, legendY + 6, 6, 4)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Raw A* path", legendX + 15, legendY)
		legendY = legendY + lineHeight

		-- Smoothed path
		love.graphics.setColor(0.2, 0.8, 0.2, 1)
		love.graphics.circle("fill", legendX + 5, legendY + 8, 3)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Smoothed path", legendX + 15, legendY)
		legendY = legendY + lineHeight

		-- Portals
		love.graphics.setColor(0.2, 0.8, 0.2, 1)
		love.graphics.rectangle("fill", legendX + 2, legendY + 6, 6, 4)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Portals", legendX + 15, legendY)
		legendY = legendY + lineHeight

		-- Corners
		love.graphics.setColor(1.0, 0.5, 0.0, 1)
		love.graphics.circle("fill", legendX + 5, legendY + 8, 4)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Corners", legendX + 15, legendY)
		legendY = legendY + lineHeight

		-- Player
		love.graphics.setColor(0.2, 0.8, 0.2, 1)
		love.graphics.circle("fill", legendX + 5, legendY + 8, 5)
		love.graphics.setColor(0, 0, 0, 1)
		love.graphics.print("Player", legendX + 15, legendY)
	end
end

function love.mousepressed(x, y, button)
	if button == 1 then -- Left mouse button
		-- Check if the clicked point is valid
		local goal = { x = x, y = y }
		local start = { x = player.x, y = player.y }

		-- First, get a raw path without advanced smoothing to show the difference
		local rawPathOptions = {
			gridSize = pathOptions.gridSize,
			portalWidth = pathOptions.portalWidth,
			cornerRadius = pathOptions.cornerRadius,
			useFunnel = false, -- Disable funnel for raw path
		}

		local rawPath, rawError =
			pathfinder.findPath(start.x, start.y, goal.x, goal.y, mainPolygon, obstacles, rawPathOptions)

		-- Then get the smoothed path with current options
		local path, error = pathfinder.findPath(start.x, start.y, goal.x, goal.y, mainPolygon, obstacles, pathOptions)

		if path then
			player:setPath(path)
			lastPath = path
			lastRawPath = rawPath -- Store raw path for comparison

			-- Generate debug information for visualization
			if #path > 2 then
				lastPortals = createSimplePortals(path)
				lastCorners = detectSimpleCorners(path)
			else
				lastPortals = {}
				lastCorners = {}
			end

			print("Path found with " .. #path .. " waypoints")
			if rawPath then
				print("Raw path had " .. #rawPath .. " waypoints")
			end
			if lastPortals and #lastPortals > 0 then
				print("Generated " .. #lastPortals .. " portals")
			end
			if lastCorners and #lastCorners > 0 then
				print("Detected " .. #lastCorners .. " corners")
			end
		else
			print("No path found to (" .. x .. ", " .. y .. "): " .. (error or "Unknown error"))
			lastPath = nil
			lastRawPath = nil
			lastPortals = nil
			lastCorners = nil
		end
	end
end

function love.keypressed(key)
	if key == "d" then
		debugMode = not debugMode
	elseif key == "n" then
		showNavMesh = not showNavMesh
		print("Navigation mesh " .. (showNavMesh and "shown" or "hidden"))
	elseif key == "p" then
		showPortals = not showPortals
		print("Portal visualization " .. (showPortals and "enabled" or "disabled"))
	elseif key == "c" then
		showCorners = not showCorners
		print("Corner visualization " .. (showCorners and "enabled" or "disabled"))
	elseif key == "f" then
		pathOptions.useFunnel = not pathOptions.useFunnel
		print("Funnel algorithm " .. (pathOptions.useFunnel and "enabled" or "disabled"))
	elseif key == "=" or key == "+" then
		-- Increase grid size (coarser grid)
		pathOptions.gridSize = math.min(pathOptions.gridSize + 5, 50)
		gridSize = pathOptions.gridSize
		navVertices, navEdges = pathfinder.getNavigationMesh(mainPolygon, obstacles, gridSize)
		print("Grid size increased to " .. pathOptions.gridSize)
	elseif key == "-" then
		-- Decrease grid size (finer grid)
		pathOptions.gridSize = math.max(pathOptions.gridSize - 5, 10)
		gridSize = pathOptions.gridSize
		navVertices, navEdges = pathfinder.getNavigationMesh(mainPolygon, obstacles, gridSize)
		print("Grid size decreased to " .. pathOptions.gridSize)
	elseif key == "1" then
		-- Decrease portal width
		pathOptions.portalWidth = math.max(pathOptions.portalWidth - 2, 5)
		print("Portal width decreased to " .. pathOptions.portalWidth)
	elseif key == "2" then
		-- Increase portal width
		pathOptions.portalWidth = math.min(pathOptions.portalWidth + 2, 30)
		print("Portal width increased to " .. pathOptions.portalWidth)
	elseif key == "3" then
		-- Decrease corner radius
		pathOptions.cornerRadius = math.max(pathOptions.cornerRadius - 1, 2)
		print("Corner radius decreased to " .. pathOptions.cornerRadius)
	elseif key == "4" then
		-- Increase corner radius
		pathOptions.cornerRadius = math.min(pathOptions.cornerRadius + 1, 15)
		print("Corner radius increased to " .. pathOptions.cornerRadius)
	elseif key == "r" then
		-- Reset to default settings
		pathOptions = {
			gridSize = 30,
			portalWidth = 15,
			cornerRadius = 8,
			useFunnel = true,
		}
		gridSize = pathOptions.gridSize
		navVertices, navEdges = pathfinder.getNavigationMesh(mainPolygon, obstacles, gridSize)
		print("Settings reset to defaults")
	end
end

-- Helper function to draw a polygon
function drawPolygon(polygon)
	local vertices = {}
	for _, point in ipairs(polygon) do
		table.insert(vertices, point.x)
		table.insert(vertices, point.y)
	end
	love.graphics.polygon("fill", vertices)
end

-- Helper function to draw polygon outline
function drawPolygonOutline(polygon)
	local vertices = {}
	for _, point in ipairs(polygon) do
		table.insert(vertices, point.x)
		table.insert(vertices, point.y)
	end
	love.graphics.setLineWidth(2)
	love.graphics.polygon("line", vertices)
	love.graphics.setLineWidth(1)
end

-- A* Pathfinding for 2D polygonal environments in Lua with grid-based navigation mesh
-- Enhanced with Funnel Algorithm for optimal path smoothing

-- Utility function to calculate distance between two points
local function distance(a, b)
	return math.sqrt((b.x - a.x) ^ 2 + (b.y - a.y) ^ 2)
end

-- Calculate cross product of two 2D vectors
local function crossProduct(a, b, c)
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
end

-- Calculate dot product of two 2D vectors
local function dotProduct(a, b, c, d)
	return (b.x - a.x) * (d.x - c.x) + (b.y - a.y) * (d.y - c.y)
end

-- Normalize a vector
local function normalize(v)
	local length = math.sqrt(v.x * v.x + v.y * v.y)
	if length > 0 then
		return { x = v.x / length, y = v.y / length }
	end
	return { x = 0, y = 0 }
end

-- Check if a point is inside a polygon using ray casting algorithm
local function isPointInPolygon(point, polygon)
	local inside = false
	local j = #polygon

	for i = 1, #polygon do
		if
			((polygon[i].y > point.y) ~= (polygon[j].y > point.y))
			and (
				point.x
				< (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y)
					+ polygon[i].x
			)
		then
			inside = not inside
		end
		j = i
	end

	return inside
end

-- Check if a point is inside any obstacle
local function isPointInObstacles(point, obstacles)
	for _, obstacle in ipairs(obstacles) do
		if isPointInPolygon(point, obstacle) then
			return true
		end
	end
	return false
end

-- Check if a point is valid (inside main polygon and not in any obstacle)
local function isValidPoint(point, mainPolygon, obstacles)
	return isPointInPolygon(point, mainPolygon) and not isPointInObstacles(point, obstacles)
end

-- Check if a point is near any polygon edge
local function isNearEdge(point, polygon, threshold)
	threshold = threshold or 10 -- Default threshold distance

	for i = 1, #polygon do
		local j = i % #polygon + 1
		local p1, p2 = polygon[i], polygon[j]

		-- Calculate distance from point to line segment
		local dx, dy = p2.x - p1.x, p2.y - p1.y
		local length = math.sqrt(dx * dx + dy * dy)

		if length > 0 then
			-- Normalize direction vector
			dx, dy = dx / length, dy / length

			-- Calculate vector from p1 to point
			local vx, vy = point.x - p1.x, point.y - p1.y

			-- Calculate projection of vector onto line
			local projection = vx * dx + vy * dy

			-- Clamp projection to line segment
			projection = math.max(0, math.min(length, projection))

			-- Calculate closest point on line
			local closestX = p1.x + projection * dx
			local closestY = p1.y + projection * dy

			-- Calculate distance from point to closest point
			local distanceToLine = math.sqrt((point.x - closestX) ^ 2 + (point.y - closestY) ^ 2)

			if distanceToLine < threshold then
				return true
			end
		end
	end

	return false
end

-- Check if a point is near any edge of any polygon
local function isNearAnyEdge(point, mainPolygon, obstacles, threshold)
	if isNearEdge(point, mainPolygon, threshold) then
		return true
	end

	for _, obstacle in ipairs(obstacles) do
		if isNearEdge(point, obstacle, threshold) then
			return true
		end
	end

	return false
end

-- Check if a line segment intersects with a polygon edge
local function doLinesIntersect(a, b, c, d)
	-- Line AB represented as a1x + b1y = c1
	local a1 = b.y - a.y
	local b1 = a.x - b.x
	local c1 = a1 * a.x + b1 * a.y

	-- Line CD represented as a2x + b2y = c2
	local a2 = d.y - c.y
	local b2 = c.x - d.x
	local c2 = a2 * c.x + b2 * c.y

	local determinant = a1 * b2 - a2 * b1

	if math.abs(determinant) < 0.0001 then
		-- Lines are parallel
		return false
	else
		local x = (b2 * c1 - b1 * c2) / determinant
		local y = (a1 * c2 - a2 * c1) / determinant

		-- Check if intersection point is on both line segments
		return (
			x >= math.min(a.x, b.x) - 0.0001
			and x <= math.max(a.x, b.x) + 0.0001
			and y >= math.min(a.y, b.y) - 0.0001
			and y <= math.max(a.y, b.y) + 0.0001
			and x >= math.min(c.x, d.x) - 0.0001
			and x <= math.max(c.x, d.x) + 0.0001
			and y >= math.min(c.y, d.y) - 0.0001
			and y <= math.max(c.y, d.y) + 0.0001
		)
	end
end

-- Check if a line segment intersects with any polygon
local function doesLineIntersectPolygon(a, b, polygon)
	for i = 1, #polygon do
		local j = i % #polygon + 1
		if doLinesIntersect(a, b, polygon[i], polygon[j]) then
			return true
		end
	end
	return false
end

-- Check if a line segment intersects with any obstacle
local function doesLineIntersectObstacles(a, b, obstacles)
	for _, obstacle in ipairs(obstacles) do
		if doesLineIntersectPolygon(a, b, obstacle) then
			return true
		end
	end
	return false
end

-- Check if there's a clear line of sight between two points
local function hasLineOfSight(a, b, mainPolygon, obstacles)
	-- Check if the line intersects any obstacle
	if doesLineIntersectObstacles(a, b, obstacles) then
		return false
	end

	-- Check if the line intersects the main polygon boundary
	if doesLineIntersectPolygon(a, b, mainPolygon) then
		-- Line intersects the boundary, but we need to check if it's going outside
		-- Sample points along the line and check if they're inside the polygon
		local samples = 10
		for i = 1, samples do
			local t = i / (samples + 1)
			local point = {
				x = a.x + (b.x - a.x) * t,
				y = a.y + (b.y - a.y) * t,
			}
			if not isPointInPolygon(point, mainPolygon) then
				return false
			end
		end
	end

	return true
end

-- Find the closest point on a line segment to a given point
local function closestPointOnSegment(point, segmentStart, segmentEnd)
	local dx = segmentEnd.x - segmentStart.x
	local dy = segmentEnd.y - segmentStart.y
	local length = dx * dx + dy * dy

	if length == 0 then
		return segmentStart
	end

	local t = ((point.x - segmentStart.x) * dx + (point.y - segmentStart.y) * dy) / length
	t = math.max(0, math.min(1, t))

	return {
		x = segmentStart.x + t * dx,
		y = segmentStart.y + t * dy,
	}
end

-- Find polygon edges that intersect with a line segment
local function findIntersectingEdges(lineStart, lineEnd, polygon)
	local intersections = {}

	for i = 1, #polygon do
		local j = i % #polygon + 1
		local edgeStart = polygon[i]
		local edgeEnd = polygon[j]

		if doLinesIntersect(lineStart, lineEnd, edgeStart, edgeEnd) then
			-- Calculate intersection point
			local a1 = lineEnd.y - lineStart.y
			local b1 = lineStart.x - lineEnd.x
			local c1 = a1 * lineStart.x + b1 * lineStart.y

			local a2 = edgeEnd.y - edgeStart.y
			local b2 = edgeStart.x - edgeEnd.x
			local c2 = a2 * edgeStart.x + b2 * edgeStart.y

			local determinant = a1 * b2 - a2 * b1
			if math.abs(determinant) > 0.0001 then
				local x = (b2 * c1 - b1 * c2) / determinant
				local y = (a1 * c2 - a2 * c1) / determinant

				table.insert(intersections, {
					point = { x = x, y = y },
					edgeStart = edgeStart,
					edgeEnd = edgeEnd,
					edgeIndex = i,
				})
			end
		end
	end

	return intersections
end

-- Create navigation portals between adjacent waypoints
local function createPortals(path, mainPolygon, obstacles, portalWidth)
	portalWidth = portalWidth or 10
	local portals = {}

	for i = 1, #path - 1 do
		local current = path[i]
		local next = path[i + 1]

		-- Find intersecting edges for the line segment
		local intersections = {}

		-- Check main polygon
		local mainIntersections = findIntersectingEdges(current, next, mainPolygon)
		for _, intersection in ipairs(mainIntersections) do
			intersection.isObstacle = false
			table.insert(intersections, intersection)
		end

		-- Check obstacles
		for _, obstacle in ipairs(obstacles) do
			local obstacleIntersections = findIntersectingEdges(current, next, obstacle)
			for _, intersection in ipairs(obstacleIntersections) do
				intersection.isObstacle = true
				table.insert(intersections, intersection)
			end
		end

		-- Create portal from intersections
		if #intersections > 0 then
			-- Sort intersections by distance from current point
			table.sort(intersections, function(a, b)
				return distance(current, a.point) < distance(current, b.point)
			end)

			-- Use the first intersection to create portal
			local intersection = intersections[1]
			local edgeDir = normalize({
				x = intersection.edgeEnd.x - intersection.edgeStart.x,
				y = intersection.edgeEnd.y - intersection.edgeStart.y,
			})

			-- Create portal perpendicular to edge
			local portalLeft = {
				x = intersection.point.x - edgeDir.y * portalWidth / 2,
				y = intersection.point.y + edgeDir.x * portalWidth / 2,
			}
			local portalRight = {
				x = intersection.point.x + edgeDir.y * portalWidth / 2,
				y = intersection.point.y - edgeDir.x * portalWidth / 2,
			}

			-- Validate portal points
			if
				isValidPoint(portalLeft, mainPolygon, obstacles)
				and isValidPoint(portalRight, mainPolygon, obstacles)
			then
				table.insert(portals, {
					left = portalLeft,
					right = portalRight,
					center = intersection.point,
				})
			else
				-- Fallback: create smaller portal or use edge endpoints
				local halfWidth = portalWidth / 4
				portalLeft = {
					x = intersection.point.x - edgeDir.y * halfWidth,
					y = intersection.point.y + edgeDir.x * halfWidth,
				}
				portalRight = {
					x = intersection.point.x + edgeDir.y * halfWidth,
					y = intersection.point.y - edgeDir.x * halfWidth,
				}

				if
					isValidPoint(portalLeft, mainPolygon, obstacles)
					and isValidPoint(portalRight, mainPolygon, obstacles)
				then
					table.insert(portals, {
						left = portalLeft,
						right = portalRight,
						center = intersection.point,
					})
				end
			end
		end
	end

	return portals
end

-- Funnel Algorithm implementation for path smoothing
local function funnelAlgorithm(start, goal, portals)
	if not portals or #portals == 0 then
		return { start, goal }
	end

	local path = { start }
	local apex = start
	local leftPortal = start
	local rightPortal = start
	local apexIndex = 0
	local leftIndex = 0
	local rightIndex = 0

	-- Add goal as final portal
	local finalPortal = { left = goal, right = goal, center = goal }
	table.insert(portals, finalPortal)

	for i = 1, #portals do
		local portal = portals[i]
		local left = portal.left
		local right = portal.right

		-- Update right side of funnel
		if rightIndex == 0 or crossProduct(apex, rightPortal, right) <= 0 then
			-- Tighten the funnel
			if rightIndex == 0 or crossProduct(apex, leftPortal, right) > 0 then
				rightPortal = right
				rightIndex = i
			else
				-- Right side crosses left side, move apex to left portal
				table.insert(path, leftPortal)
				apex = leftPortal
				apexIndex = leftIndex

				-- Reset funnel
				leftPortal = apex
				rightPortal = apex
				leftIndex = apexIndex
				rightIndex = apexIndex

				-- Restart from current portal
				i = apexIndex
				goto continue
			end
		end

		-- Update left side of funnel
		if leftIndex == 0 or crossProduct(apex, leftPortal, left) >= 0 then
			-- Tighten the funnel
			if leftIndex == 0 or crossProduct(apex, rightPortal, left) < 0 then
				leftPortal = left
				leftIndex = i
			else
				-- Left side crosses right side, move apex to right portal
				table.insert(path, rightPortal)
				apex = rightPortal
				apexIndex = rightIndex

				-- Reset funnel
				leftPortal = apex
				rightPortal = apex
				leftIndex = apexIndex
				rightIndex = apexIndex

				-- Restart from current portal
				i = apexIndex
				goto continue
			end
		end

		::continue::
	end

	-- Add goal if not already added
	if path[#path] ~= goal then
		table.insert(path, goal)
	end

	return path
end

-- Enhanced path validation with corner detection
local function validatePathSegment(start, end_, mainPolygon, obstacles, tolerance)
	tolerance = tolerance or 1.0

	-- Check basic line of sight
	if not hasLineOfSight(start, end_, mainPolygon, obstacles) then
		return false
	end

	-- Sample points along the path for additional validation
	local samples = math.max(5, math.floor(distance(start, end_) / 10))
	for i = 1, samples do
		local t = i / (samples + 1)
		local point = {
			x = start.x + (end_.x - start.x) * t,
			y = start.y + (end_.y - start.y) * t,
		}

		if not isValidPoint(point, mainPolygon, obstacles) then
			return false
		end

		-- Check if point is too close to any edge
		if isNearAnyEdge(point, mainPolygon, obstacles, tolerance) then
			return false
		end
	end

	return true
end

-- Corner detection and handling
local function detectCorners(path, mainPolygon, obstacles)
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

-- Optimize path around corners
local function optimizeCorners(path, corners, mainPolygon, obstacles, cornerRadius)
	cornerRadius = cornerRadius or 5
	local optimizedPath = {}

	local i = 1
	while i <= #path do
		local isCorner = false
		local corner = nil

		-- Check if current point is a corner
		for _, c in ipairs(corners) do
			if c.index == i then
				isCorner = true
				corner = c
				break
			end
		end

		if isCorner and corner.isSharp and i > 1 and i < #path then
			-- Create smooth corner
			local prev = path[i - 1]
			local current = path[i]
			local next = path[i + 1]

			-- Calculate approach and exit vectors
			local approachDir = normalize({ x = current.x - prev.x, y = current.y - prev.y })
			local exitDir = normalize({ x = next.x - current.x, y = next.y - current.y })

			-- Create corner points
			local cornerStart = {
				x = current.x - approachDir.x * cornerRadius,
				y = current.y - approachDir.y * cornerRadius,
			}
			local cornerEnd = {
				x = current.x + exitDir.x * cornerRadius,
				y = current.y + exitDir.y * cornerRadius,
			}

			-- Validate corner points
			if
				validatePathSegment(prev, cornerStart, mainPolygon, obstacles)
				and validatePathSegment(cornerStart, cornerEnd, mainPolygon, obstacles)
				and validatePathSegment(cornerEnd, next, mainPolygon, obstacles)
			then
				table.insert(optimizedPath, cornerStart)
				table.insert(optimizedPath, cornerEnd)
			else
				table.insert(optimizedPath, current)
			end
		else
			table.insert(optimizedPath, path[i])
		end

		i = i + 1
	end

	return optimizedPath
end

-- QuadTree implementation for spatial partitioning
local function createQuadTree(bounds, maxPoints, maxDepth)
	return {
		bounds = bounds, -- {minX, minY, maxX, maxY}
		points = {},
		children = nil,
		maxPoints = maxPoints or 10,
		maxDepth = maxDepth or 5,
		depth = 0,

		-- Insert a point into the quadtree
		insert = function(self, point, data)
			if not self:containsPoint(point) then
				return false
			end

			if #self.points < self.maxPoints or self.depth >= self.maxDepth then
				table.insert(self.points, { point = point, data = data })
				return true
			end

			-- Subdivide if needed
			if not self.children then
				self:subdivide()
			end

			-- Try to insert into children
			for _, child in ipairs(self.children) do
				if child:insert(point, data) then
					return true
				end
			end

			-- If we couldn't insert into children (shouldn't happen)
			table.insert(self.points, { point = point, data = data })
			return true
		end,

		-- Subdivide the quadtree into four children
		subdivide = function(self)
			local minX, minY, maxX, maxY = self.bounds[1], self.bounds[2], self.bounds[3], self.bounds[4]
			local midX, midY = (minX + maxX) / 2, (minY + maxY) / 2

			self.children = {
				createQuadTree({ minX, minY, midX, midY }, self.maxPoints, self.maxDepth),
				createQuadTree({ midX, minY, maxX, midY }, self.maxPoints, self.maxDepth),
				createQuadTree({ minX, midY, midX, maxY }, self.maxPoints, self.maxDepth),
				createQuadTree({ midX, midY, maxX, maxY }, self.maxPoints, self.maxDepth),
			}

			-- Set depth for children
			for _, child in ipairs(self.children) do
				child.depth = self.depth + 1
			end

			-- Redistribute points to children
			local points = self.points
			self.points = {}

			for _, pointData in ipairs(points) do
				self:insert(pointData.point, pointData.data)
			end
		end,

		-- Check if a point is within the quadtree bounds
		containsPoint = function(self, point)
			local minX, minY, maxX, maxY = self.bounds[1], self.bounds[2], self.bounds[3], self.bounds[4]
			return point.x >= minX and point.x <= maxX and point.y >= minY and point.y <= maxY
		end,

		-- Query points within a radius
		queryRadius = function(self, center, radius)
			local result = {}

			-- Check if the search area intersects this quad
			local minX, minY, maxX, maxY = self.bounds[1], self.bounds[2], self.bounds[3], self.bounds[4]
			if
				center.x + radius < minX
				or center.x - radius > maxX
				or center.y + radius < minY
				or center.y - radius > maxY
			then
				return result
			end

			-- Check points at this level
			for _, pointData in ipairs(self.points) do
				local point = pointData.point
				if distance(center, point) <= radius then
					table.insert(result, pointData)
				end
			end

			-- Check children if they exist
			if self.children then
				for _, child in ipairs(self.children) do
					local childResults = child:queryRadius(center, radius)
					for _, pointData in ipairs(childResults) do
						table.insert(result, pointData)
					end
				end
			end

			return result
		end,
	}
end

-- Min-heap priority queue implementation
local function createPriorityQueue()
	local heap = {}

	local function parentIndex(i)
		return math.floor(i / 2)
	end
	local function leftChildIndex(i)
		return i * 2
	end
	local function rightChildIndex(i)
		return i * 2 + 1
	end

	return {
		heap = heap,

		isEmpty = function(self)
			return #self.heap == 0
		end,

		push = function(self, item, priority)
			table.insert(self.heap, { item = item, priority = priority })
			local i = #self.heap

			-- Bubble up
			while i > 1 do
				local parent = parentIndex(i)
				if self.heap[parent].priority <= self.heap[i].priority then
					break
				end

				-- Swap with parent
				self.heap[parent], self.heap[i] = self.heap[i], self.heap[parent]
				i = parent
			end
		end,

		pop = function(self)
			if self:isEmpty() then
				return nil
			end

			local top = self.heap[1]
			local last = table.remove(self.heap)

			if #self.heap > 0 then
				self.heap[1] = last
				local i = 1

				-- Bubble down
				while true do
					local left = leftChildIndex(i)
					local right = rightChildIndex(i)
					local smallest = i

					if left <= #self.heap and self.heap[left].priority < self.heap[smallest].priority then
						smallest = left
					end

					if right <= #self.heap and self.heap[right].priority < self.heap[smallest].priority then
						smallest = right
					end

					if smallest == i then
						break
					end

					-- Swap with smallest child
					self.heap[i], self.heap[smallest] = self.heap[smallest], self.heap[i]
					i = smallest
				end
			end

			return top.item
		end,

		update = function(self, item, newPriority)
			for i, node in ipairs(self.heap) do
				if node.item == item then
					local oldPriority = node.priority
					node.priority = newPriority

					if newPriority < oldPriority then
						-- Bubble up
						while i > 1 do
							local parent = parentIndex(i)
							if self.heap[parent].priority <= self.heap[i].priority then
								break
							end

							-- Swap with parent
							self.heap[parent], self.heap[i] = self.heap[i], self.heap[parent]
							i = parent
						end
					else
						-- Bubble down
						while true do
							local left = leftChildIndex(i)
							local right = rightChildIndex(i)
							local smallest = i

							if left <= #self.heap and self.heap[left].priority < self.heap[smallest].priority then
								smallest = left
							end

							if right <= #self.heap and self.heap[right].priority < self.heap[smallest].priority then
								smallest = right
							end

							if smallest == i then
								break
							end

							-- Swap with smallest child
							self.heap[i], self.heap[smallest] = self.heap[smallest], self.heap[i]
							i = smallest
						end
					end

					return true
				end
			end

			return false
		end,
	}
end

-- Generate a grid-based navigation mesh with adaptive grid sizing
local function generateGridNavMesh(mainPolygon, obstacles, baseGridSize)
	baseGridSize = baseGridSize or 20 -- Default grid size
	local edgeThreshold = baseGridSize / 2 -- Distance to stay away from edges

	-- Find bounds of the main polygon
	local minX, minY = math.huge, math.huge
	local maxX, maxY = -math.huge, -math.huge

	for _, point in ipairs(mainPolygon) do
		minX = math.min(minX, point.x)
		minY = math.min(minY, point.y)
		maxX = math.max(maxX, point.x)
		maxY = math.max(maxY, point.y)
	end

	-- Calculate area and determine appropriate grid size
	local area = (maxX - minX) * (maxY - minY)
	local gridSize = baseGridSize

	-- Adjust grid size based on area
	if area > 1000000 then -- Very large area
		gridSize = baseGridSize * 2
	elseif area < 10000 then -- Small area
		gridSize = baseGridSize / 2
	end

	-- Add padding
	minX = minX + gridSize
	minY = minY + gridSize
	maxX = maxX - gridSize
	maxY = maxY - gridSize

	-- Create quadtree for spatial partitioning
	local quadtree = createQuadTree({ minX, minY, maxX, maxY })
	local gridPoints = {}

	-- Generate grid points
	for x = minX, maxX, gridSize do
		for y = minY, maxY, gridSize do
			local point = { x = x, y = y }

			-- Only add points that are valid and not too close to edges
			if
				isValidPoint(point, mainPolygon, obstacles)
				and not isNearAnyEdge(point, mainPolygon, obstacles, edgeThreshold)
			then
				table.insert(gridPoints, point)
				quadtree:insert(point, #gridPoints)
			end
		end
	end

	return gridPoints, quadtree
end

-- Create a node for A* algorithm
local function createNode(position)
	return {
		position = position,
		g = 0, -- Cost from start to current node
		h = 0, -- Heuristic (estimated cost from current to goal)
		f = 0, -- Total cost (g + h)
		parent = nil, -- Reference to parent node
	}
end

-- A* pathfinding algorithm with priority queue and quadtree
local function findPath(start, goal, mainPolygon, obstacles, gridSize)
	gridSize = gridSize or 20 -- Default grid size

	-- Check if start and goal are valid positions
	if not isValidPoint(start, mainPolygon, obstacles) then
		return nil, "Start position is invalid"
	end

	if not isValidPoint(goal, mainPolygon, obstacles) then
		return nil, "Goal position is invalid"
	end

	-- Direct path check - if there's a clear line of sight, return direct path
	if hasLineOfSight(start, goal, mainPolygon, obstacles) then
		return { start, goal }
	end

	-- Generate grid-based navigation mesh with quadtree
	local gridPoints, quadtree = generateGridNavMesh(mainPolygon, obstacles, gridSize)

	-- Add start and goal to the grid points
	table.insert(gridPoints, start)
	local startIdx = #gridPoints

	table.insert(gridPoints, goal)
	local goalIdx = #gridPoints

	-- Insert start and goal into quadtree
	quadtree:insert(start, startIdx)
	quadtree:insert(goal, goalIdx)

	-- Create adjacency list (graph) using quadtree for neighbor finding
	local graph = {}
	local maxConnectionDistance = gridSize * 2 -- Maximum distance for connections

	for i, point in ipairs(gridPoints) do
		graph[i] = {}

		-- Query nearby points using quadtree
		local nearbyPoints = quadtree:queryRadius(point, maxConnectionDistance)

		for _, neighborData in ipairs(nearbyPoints) do
			local neighborIdx = neighborData.data
			local neighbor = neighborData.point

			if i ~= neighborIdx then
				local dist = distance(point, neighbor)
				if dist <= maxConnectionDistance and hasLineOfSight(point, neighbor, mainPolygon, obstacles) then
					table.insert(graph[i], { idx = neighborIdx, cost = dist })
				end
			end
		end
	end

	-- Check if start or goal has no connections
	if #graph[startIdx] == 0 then
		return nil, "Start position has no valid connections"
	end

	if #graph[goalIdx] == 0 then
		return nil, "Goal position has no valid connections"
	end

	-- Initialize priority queue for open set
	local openQueue = createPriorityQueue()
	local openSet = {} -- For quick lookup
	local closedSet = {}

	-- Create start node
	local startNode = createNode(gridPoints[startIdx])
	startNode.h = distance(gridPoints[startIdx], gridPoints[goalIdx])
	startNode.f = startNode.h

	-- Add start node to open set
	openQueue:push(startIdx, startNode.f)
	openSet[startIdx] = startNode

	local iterations = 0
	local maxIterations = 5000 -- Increased safety limit

	while not openQueue:isEmpty() and iterations < maxIterations do
		iterations = iterations + 1

		-- Get node with lowest f score from priority queue
		local currentIdx = openQueue:pop()
		local currentNode = openSet[currentIdx]

		-- If goal is reached
		if currentIdx == goalIdx then
			-- Reconstruct path
			local path = {}
			local current = currentNode

			while current do
				table.insert(path, 1, current.position)
				current = current.parent
			end

			return path
		end

		-- Move current node from open to closed set
		openSet[currentIdx] = nil
		closedSet[currentIdx] = currentNode

		-- Check all neighbors
		for _, neighbor in ipairs(graph[currentIdx]) do
			local neighborIdx = neighbor.idx
			local cost = neighbor.cost

			-- Skip if neighbor is in closed set
			if closedSet[neighborIdx] then
				goto continue
			end

			local neighborPos = gridPoints[neighborIdx]
			local tentativeG = currentNode.g + cost

			-- If neighbor is not in open set or has a better g score
			if not openSet[neighborIdx] then
				local neighborNode = createNode(neighborPos)
				neighborNode.parent = currentNode
				neighborNode.g = tentativeG
				neighborNode.h = distance(neighborPos, gridPoints[goalIdx])
				neighborNode.f = neighborNode.g + neighborNode.h

				openSet[neighborIdx] = neighborNode
				openQueue:push(neighborIdx, neighborNode.f)
			elseif tentativeG < openSet[neighborIdx].g then
				local neighborNode = openSet[neighborIdx]
				neighborNode.parent = currentNode
				neighborNode.g = tentativeG
				neighborNode.f = tentativeG + neighborNode.h

				openQueue:update(neighborIdx, neighborNode.f)
			end

			::continue::
		end
	end

	if iterations >= maxIterations then
		return nil, "Pathfinding exceeded maximum iterations"
	else
		return nil, "No path found after " .. iterations .. " iterations"
	end
end

-- Enhanced path smoothing with Funnel Algorithm
local function smoothPathWithFunnel(path, mainPolygon, obstacles, options)
	if not path or #path < 3 then
		return path
	end

	options = options or {}
	local portalWidth = options.portalWidth or 15
	local cornerRadius = options.cornerRadius or 8
	local useFunnel = options.useFunnel ~= false -- Default to true

	-- Step 1: Basic line-of-sight smoothing
	local basicSmoothed = { path[1] }
	local currentPoint = 1

	while currentPoint < #path do
		-- Try to find the furthest point with line of sight
		local furthestVisible = currentPoint

		for i = #path, currentPoint + 1, -1 do
			if validatePathSegment(path[currentPoint], path[i], mainPolygon, obstacles) then
				furthestVisible = i
				break
			end
		end

		currentPoint = furthestVisible
		table.insert(basicSmoothed, path[currentPoint])
	end

	-- Step 2: Apply Funnel Algorithm if enabled and path is complex enough
	local funnelSmoothed = basicSmoothed
	if useFunnel and #basicSmoothed > 2 then
		local portals = createPortals(basicSmoothed, mainPolygon, obstacles, portalWidth)
		if #portals > 0 then
			funnelSmoothed = funnelAlgorithm(basicSmoothed[1], basicSmoothed[#basicSmoothed], portals)
		end
	end

	-- Step 3: Corner optimization
	local corners = detectCorners(funnelSmoothed, mainPolygon, obstacles)
	local finalPath = optimizeCorners(funnelSmoothed, corners, mainPolygon, obstacles, cornerRadius)

	-- Step 4: Final validation and cleanup
	local validatedPath = { finalPath[1] }
	for i = 2, #finalPath do
		if validatePathSegment(validatedPath[#validatedPath], finalPath[i], mainPolygon, obstacles) then
			table.insert(validatedPath, finalPath[i])
		else
			-- If validation fails, fall back to previous valid point
			-- and try to find an intermediate point
			local lastValid = validatedPath[#validatedPath]
			local target = finalPath[i]

			-- Try to find a valid intermediate point
			local found = false
			for j = i - 1, 2, -1 do
				if validatePathSegment(lastValid, finalPath[j], mainPolygon, obstacles) then
					table.insert(validatedPath, finalPath[j])
					found = true
					break
				end
			end

			if not found then
				-- If no intermediate point works, add the target anyway
				-- (this shouldn't happen with proper validation)
				table.insert(validatedPath, target)
			end
		end
	end

	return validatedPath
end

-- Function to find path between two points with enhanced smoothing
local function findPathBetweenPoints(startX, startY, goalX, goalY, mainPolygon, obstacles, options)
	-- Validate inputs
	if type(startX) ~= "number" or type(startY) ~= "number" or type(goalX) ~= "number" or type(goalY) ~= "number" then
		return nil, "Invalid coordinates: must be numbers"
	end

	if not mainPolygon or #mainPolygon < 3 then
		return nil, "Invalid main polygon: must have at least 3 points"
	end

	options = options or {}
	local gridSize = options.gridSize or 20

	local start = { x = startX, y = startY }
	local goal = { x = goalX, y = goalY }

	local path, error = findPath(start, goal, mainPolygon, obstacles, gridSize)
	if path then
		return smoothPathWithFunnel(path, mainPolygon, obstacles, options)
	end
	return nil, error
end

-- Function to get the navigation mesh for visualization
local function getNavigationMesh(mainPolygon, obstacles, gridSize)
	local gridPoints = generateGridNavMesh(mainPolygon, obstacles, gridSize)
	local edges = {}
	local maxConnectionDistance = gridSize * 2

	for i, point1 in ipairs(gridPoints) do
		for j, point2 in ipairs(gridPoints) do
			if i < j then
				local dist = distance(point1, point2)
				if dist <= maxConnectionDistance and hasLineOfSight(point1, point2, mainPolygon, obstacles) then
					table.insert(edges, { point1, point2 })
				end
			end
		end
	end

	return gridPoints, edges
end

-- Return the module
return {
	findPath = findPathBetweenPoints,
	isPointInPolygon = isPointInPolygon,
	hasLineOfSight = hasLineOfSight,
	getNavigationMesh = getNavigationMesh,
	validatePathSegment = validatePathSegment,
	detectCorners = detectCorners,
	funnelAlgorithm = funnelAlgorithm,
	createPortals = createPortals,
}

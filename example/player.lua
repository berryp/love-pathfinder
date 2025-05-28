-- player.lua
local Player = {}

function Player:new()
	local player = {
		x = 100,
		y = 100,
		width = 15,
		height = 30,
		speed = 150,
		path = nil,
		currentPathIndex = 1,
		targetX = nil,
		targetY = nil,
		isMoving = false,
	}
	setmetatable(player, { __index = self })
	return player
end

function Player:update(dt)
	if self.isMoving and self.path and self.currentPathIndex <= #self.path then
		local target = self.path[self.currentPathIndex]
		local dx = target.x - self.x
		local dy = target.y - self.y
		local distance = math.sqrt(dx * dx + dy * dy)

		-- If we're close enough to the current waypoint, move to the next one
		if distance < 5 then
			self.currentPathIndex = self.currentPathIndex + 1

			-- If we've reached the end of the path, stop moving
			if self.currentPathIndex > #self.path then
				self.isMoving = false
				return
			end
		else
			-- Move towards the current waypoint
			local angle = math.atan2(dy, dx)
			local moveX = math.cos(angle) * self.speed * dt
			local moveY = math.sin(angle) * self.speed * dt

			-- Don't overshoot the target
			if math.abs(moveX) > math.abs(dx) then
				moveX = dx
			end
			if math.abs(moveY) > math.abs(dy) then
				moveY = dy
			end

			self.x = self.x + moveX
			self.y = self.y + moveY
		end
	end
end

function Player:draw()
	-- Draw the player as a white rectangle
	love.graphics.setColor(1, 1, 1)
	love.graphics.rectangle("fill", self.x - self.width / 2, self.y - self.height / 2, self.width, self.height)

	-- Draw the path if it exists
	if self.path then
		love.graphics.setColor(1, 1, 1, 0.5)
		for i = 1, #self.path - 1 do
			love.graphics.line(self.path[i].x, self.path[i].y, self.path[i + 1].x, self.path[i + 1].y)
		end

		-- Draw waypoints as small circles
		love.graphics.setColor(1, 0.5, 0, 0.8)
		for i = 1, #self.path do
			love.graphics.circle("fill", self.path[i].x, self.path[i].y, 5)
		end

		-- Highlight the current target waypoint
		if self.currentPathIndex <= #self.path then
			love.graphics.setColor(0, 1, 0, 0.8)
			love.graphics.circle("fill", self.path[self.currentPathIndex].x, self.path[self.currentPathIndex].y, 8)
		end
	end
end

function Player:setPath(path)
	self.path = path
	self.currentPathIndex = 1
	self.isMoving = true
end

return Player

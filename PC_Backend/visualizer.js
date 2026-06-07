class RobotVisualizer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.currentPos = 0;
        this.targetPos = 0;
        
        window.addEventListener('resize', () => this.resize());
        this.resize();
    }

    resize() {
        const container = this.canvas.parentElement;
        this.canvas.width = container.clientWidth;
        this.canvas.height = container.clientHeight;
        this.draw();
    }

    update(currentPos, targetPos) {
        this.currentPos = currentPos;
        this.targetPos = targetPos;
        this.draw();
    }

    draw() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const centerX = width / 2;
        const centerY = height / 2;
        const radius = Math.min(width, height) * 0.35;

        ctx.clearRect(0, 0, width, height);

        // Draw Path Reference
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Draw Degrees Markers
        ctx.font = '8px Inter';
        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.textAlign = 'center';
        for(let i=0; i<360; i+=45) {
            const rad = (i - 90) * Math.PI / 180;
            const x = centerX + (radius + 15) * Math.cos(rad);
            const y = centerY + (radius + 15) * Math.sin(rad);
            ctx.fillText(i + '°', x, y);
        }

        // Draw Target Ghost
        this.drawArm(centerX, centerY, radius, this.targetPos, 'rgba(255, 255, 255, 0.2)', true);

        // Draw Actual Arm
        this.drawArm(centerX, centerY, radius, this.currentPos, '#00f2ff', false);

        // Draw Motor Shaft (Center Hub)
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, Math.PI * 2);
        ctx.fillStyle = '#1a1a1a';
        ctx.fill();
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    drawArm(cx, cy, length, angleDeg, color, isGhost) {
        const ctx = this.ctx;
        // Adjust for 0 deg being UP (standard in many robot coords, or adjust to match firmware)
        // Firmware usually has 0 as a start point. Let's assume 0 is top.
        const rad = (angleDeg - 90) * Math.PI / 180;
        const x = cx + length * Math.cos(rad);
        const y = cy + length * Math.sin(rad);

        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(x, y);
        ctx.strokeStyle = color;
        ctx.lineWidth = isGhost ? 1 : 4;
        if(isGhost) ctx.setLineDash([5, 5]);
        else ctx.setLineDash([]);
        ctx.lineCap = 'round';
        ctx.stroke();

        // End effector point
        ctx.beginPath();
        ctx.arc(x, y, isGhost ? 3 : 5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
    }
}

class GripperVisualizer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.isDown = false;
        this.isOpen = true;
        window.addEventListener('resize', () => this.resize());
        this.resize();
    }

    resize() {
        this.canvas.width  = this.canvas.offsetWidth  || 72;
        this.canvas.height = this.canvas.offsetHeight || 130;
        this.draw();
    }

    update(isDown, isOpen) {
        this.isDown = isDown;
        this.isOpen = isOpen;
        this.draw();
    }

    draw() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        ctx.clearRect(0, 0, width, height);

        const cx = width / 2;
        const armColor  = this.isDown ? '#00f2ff' : '#39ff14';
        const clawColor = this.isOpen  ? '#39ff14' : '#00f2ff';

        // Title
        ctx.font = '7px Inter';
        ctx.fillStyle = 'rgba(255,255,255,0.3)';
        ctx.textAlign = 'center';
        ctx.fillText('GRIPPER', cx, 10);

        // UP/DOWN badge
        ctx.font = 'bold 7px JetBrains Mono';
        ctx.fillStyle = armColor;
        ctx.fillText(this.isDown ? 'DOWN' : 'UP', cx, 21);

        // Mounting bracket
        const mountY = 26;
        ctx.fillStyle = 'rgba(255,255,255,0.15)';
        ctx.fillRect(cx - 14, mountY, 28, 4);

        // Arm body
        const armTopY   = mountY + 4;
        const armLen    = this.isDown ? height * 0.42 : height * 0.22;
        const armW      = 7;
        ctx.fillStyle   = armColor;
        ctx.shadowColor = this.isDown ? '#00f2ff' : '#39ff14';
        ctx.shadowBlur  = 8;
        ctx.fillRect(cx - armW / 2, armTopY, armW, armLen);
        ctx.shadowBlur  = 0;

        // Claw base bar
        const clawBaseY = armTopY + armLen;
        const barW = 22;
        ctx.fillStyle   = clawColor;
        ctx.shadowColor = this.isOpen ? '#39ff14' : '#00f2ff';
        ctx.shadowBlur  = 8;
        ctx.fillRect(cx - barW / 2, clawBaseY, barW, 3);

        // Claw fingers
        const fingerLen   = 13;
        const spread      = this.isOpen ? 0.38 : 0.05;
        ctx.strokeStyle   = clawColor;
        ctx.lineWidth     = 3;
        ctx.lineCap       = 'round';

        ctx.beginPath();
        ctx.moveTo(cx - barW / 2 + 3, clawBaseY + 3);
        ctx.lineTo(cx - barW / 2 + 3 - Math.sin(spread) * fingerLen,
                   clawBaseY + 3 + Math.cos(spread) * fingerLen);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(cx + barW / 2 - 3, clawBaseY + 3);
        ctx.lineTo(cx + barW / 2 - 3 + Math.sin(spread) * fingerLen,
                   clawBaseY + 3 + Math.cos(spread) * fingerLen);
        ctx.stroke();

        ctx.shadowBlur = 0;

        // OPEN/CLOSED label
        const labelY = Math.min(clawBaseY + fingerLen + 14, height - 4);
        ctx.font = 'bold 7px JetBrains Mono';
        ctx.fillStyle = clawColor;
        ctx.fillText(this.isOpen ? 'OPEN' : 'CLOSED', cx, labelY);
    }
}

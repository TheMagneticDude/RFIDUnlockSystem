import discord
from discord.ext import commands
from discord import app_commands
import os

#========================= Constants =========================
#will fill in after to avoid secret being in repo
botToken = '';
#grab token from local env variable
botToken = os.getenv("DISCORD_TOKEN");

GUILD_ID = discord.Object(id=1490450047085838446);



 


#========================= Class =========================
class Client(commands.Bot):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        try:
            synced = await self.tree.sync(guild=GUILD_ID)
            print(f"Synced {len(synced)} command(s) to guild.")
        except Exception as e:
            print(e)
        
    async def on_message(self, message):
        if message.author == self.user:
            return

        if message.content.startswith('!sesame'):
            await message.channel.send(
                f'Command Recieved: {message.author} executed: {message.content}'
            )

        await self.process_commands(message)



 




unlocked = False;

embedColor = discord.Color.red();
embedTitle = "Door State: Locked 🔒"
#discord.Color.green();
#"Door State: Unlocked 🔓"


doorembed = discord.Embed(title = embedTitle, description = "TestState", color = discord.Color.green());
doorembed.set_thumbnail(url="https://cdn.discordapp.com/attachments/805648700365209631/1419696609620398213/Magnets_symbol128x128.png?ex=69d3c780&is=69d27600&hm=e2d3edfc0e19da48f838cb84a7f7c570af1714d7c5ab316d3ed9cb4c9f89f798&");
   

def updateDoorEmbed(:
    global doorembed
    doorembed = discord.Embed(title = embedTitle, description = "TestState", color = discord.Color.green());
    doorembed.set_thumbnail(url="https://cdn.discordapp.com/attachments/805648700365209631/1419696609620398213/Magnets_symbol128x128.png?ex=69d3c780&is=69d27600&hm=e2d3edfc0e19da48f838cb84a7f7c570af1714d7c5ab316d3ed9cb4c9f89f798&");  


#discord UI button component
class ViewButton(discord.ui.View):
    @discord.ui.button(label="Unlock", style=discord.ButtonStyle.primary, emoji="🧲")
    async def button_callback(self, button, interaction):
        await button.response.send_message("Button detected!");
        #update embed on button press
        unlocked = not unlocked;
        
        if(unlocked):
            embedColor = discord.Color.green();
            embedTitle = "Unlocked 🔓"
        else:
            embedColor = discord.Color.red();
            embedTitle = "Door State: Locked 🔒"
        updateDoorEmbed()
        await interaction.response.edit_message(embed=doorembed)
 
    
    
    

#========================= Init =========================
intents = discord.Intents.default();
intents.message_content = True;
client = Client(command_prefix = "!sesame", intents = intents);





#========================= Commands =========================
@client.tree.command(name="test", description="test", guild = GUILD_ID)
async def test(interaction: discord.Interaction):
    await interaction.response.send_message("Test");
    
@client.tree.command(name="door", description="Controls Door", guild = GUILD_ID)
async def doorCommand(interaction: discord.Interaction):
    await interaction.response.send_message(embed = doorembed, view=ViewButton());














#run bot
client.run(botToken);
